#include "can_logger.h"
#include "esphome/core/log.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

namespace esphome {
namespace can_logger {

static const char *const TAG = "can_logger";

// Internal-RAM staging ring depth (power of two). The IRAM ISR writes here and
// the high-priority capture task drains it within microseconds, so this only
// has to cover scheduling jitter, not a whole burst. ~2048 * 21 B ~= 43 KB DRAM.
static const uint32_t STAGING_FRAMES = 2048;

// ---- small helpers --------------------------------------------------------
static uint32_t round_up_pow2(uint32_t v) {
  uint32_t p = 1;
  while (p < v)
    p <<= 1;
  return p;
}

// writer-task state kept in the .cpp to keep the header lean
namespace {
FILE *g_seg_file = nullptr;       // free-run rotating segment
FILE *g_flash_file = nullptr;     // carved UDS flash session
char g_flash_name[64] = {0};
uint32_t g_seg_bytes_written = 0;
uint32_t g_flash_bytes_written = 0;
uint64_t g_flash_end_deadline = 0;  // 0 = not in post-roll
// pre-roll ring (PSRAM): recent records to prepend to a flash file
Record *g_pre = nullptr;
uint32_t g_pre_cap = 0, g_pre_head = 0, g_pre_count = 0;
httpd_handle_t g_httpd = nullptr;
std::string g_sd_path;            // copy for http handlers
char g_last_flash_name[64] = {0};
uint32_t g_boot_seg = 0;          // first segment index of this boot
}  // namespace

// Next unused segNNNN index on the card, so a reboot (power blip, OTA) never
// truncates the previous capture.
static uint32_t next_free_seg_index(const std::string &dir) {
  uint32_t next = 0;
  DIR *d = opendir(dir.c_str());
  if (d == nullptr)
    return 0;
  struct dirent *e;
  while ((e = readdir(d))) {
    unsigned n;
    if (sscanf(e->d_name, "seg%u.canbin", &n) == 1 && n + 1 > next)
      next = n + 1;
  }
  closedir(d);
  return next;
}

// ---- PSRAM ring (SPSC: capture_task producer / writer_task consumer) ------
bool CanLogger::ring_push_(const Record &r) {
  uint32_t head = ring_head_.load(std::memory_order_relaxed);
  uint32_t tail = ring_tail_.load(std::memory_order_acquire);
  if (((head + 1) & ring_mask_) == (tail & ring_mask_))
    return false;  // full: caller counts this as loss
  ring_[head & ring_mask_] = r;
  ring_head_.store(head + 1, std::memory_order_release);
  return true;
}

bool CanLogger::ring_pop_(Record &out) {
  uint32_t tail = ring_tail_.load(std::memory_order_relaxed);
  uint32_t head = ring_head_.load(std::memory_order_acquire);
  if ((tail & ring_mask_) == (head & ring_mask_))
    return false;  // empty
  out = ring_[tail & ring_mask_];
  ring_tail_.store(tail + 1, std::memory_order_release);
  return true;
}

// ---- staging ring consumer (capture task, cache enabled) ------------------
bool CanLogger::pop_stage_(RawFrame &out) {
  uint32_t tail = stage_tail_.load(std::memory_order_relaxed);
  uint32_t head = stage_head_.load(std::memory_order_acquire);
  if ((tail & stage_mask_) == (head & stage_mask_))
    return false;
  out = stage_[tail & stage_mask_];
  stage_tail_.store(tail + 1, std::memory_order_release);
  return true;
}

// ---- RX ISR (IRAM, cache-safe): HW FIFO -> internal-RAM staging ring -------
// Everything this touches is internal RAM or IRAM: the staging buffer, the
// atomics, esp_timer_get_time() and twai_node_receive_from_isr(). It never
// touches PSRAM or flash-resident code, so it keeps running with cache off.
IRAM_ATTR bool CanLogger::on_rx_(twai_node_handle_t node,
                                 const twai_rx_done_event_data_t *edata, void *ctx) {
  CanLogger *self = (CanLogger *) ctx;
  twai_frame_t frame = {};
  uint8_t buf[8];
  frame.buffer = buf;
  frame.buffer_len = sizeof(buf);
  if (twai_node_receive_from_isr(node, &frame) != ESP_OK)
    return false;

  uint32_t head = self->stage_head_.load(std::memory_order_relaxed);
  uint32_t tail = self->stage_tail_.load(std::memory_order_acquire);
  if (((head + 1) & self->stage_mask_) == (tail & self->stage_mask_)) {
    // staging full: the capture task fell behind. Lost frame = integrity fault.
    self->stage_overrun_.fetch_add(1, std::memory_order_relaxed);
    self->gap_pending_.store(true, std::memory_order_relaxed);
    return false;
  }
  RawFrame &rf = self->stage_[head & self->stage_mask_];
  rf.ts_us = (uint64_t) esp_timer_get_time();
  uint32_t id = frame.header.id & 0x1FFFFFFFu;
  if (frame.header.ide)
    id |= (1u << 31);
  if (frame.header.rtr)
    id |= (1u << 30);
  rf.id = id;
  rf.dlc = frame.header.dlc <= 8 ? (uint8_t) frame.header.dlc : 8;
  for (uint8_t i = 0; i < rf.dlc; i++)
    rf.data[i] = buf[i];
  self->stage_head_.store(head + 1, std::memory_order_release);
  return false;  // no task woken directly; capture task polls
}

IRAM_ATTR bool CanLogger::on_err_(twai_node_handle_t node,
                                  const twai_error_event_data_t *edata, void *ctx) {
  CanLogger *self = (CanLogger *) ctx;
  self->bus_errors_.fetch_add(1, std::memory_order_relaxed);
  // NOTE: if your ESP-IDF exposes an explicit RX data-overrun flag in
  // twai_error_event_data_t, OR it into overrun_total_ + gap_pending_ here so a
  // HW-FIFO overrun also fails the integrity gate. The staging/PSRAM overflow
  // paths already catch the common "software too slow" case authoritatively.
  return false;
}

// ---- setup ----------------------------------------------------------------
void CanLogger::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 CAN logger (esp_twai node API)...");
  g_sd_path = sd_path_;

  // staging ring in INTERNAL RAM (ISR-reachable with cache off)
  stage_cap_ = round_up_pow2(STAGING_FRAMES);
  stage_mask_ = stage_cap_ - 1;
  stage_ = (RawFrame *) heap_caps_malloc((size_t) stage_cap_ * sizeof(RawFrame),
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (stage_ == nullptr) {
    ESP_LOGE(TAG, "staging ring alloc failed");
    this->mark_failed();
    return;
  }

  // PSRAM ring
  uint64_t bytes = (uint64_t) ring_mb_ * 1024 * 1024;
  ring_cap_ = round_up_pow2((uint32_t)(bytes / sizeof(Record)));
  ring_mask_ = ring_cap_ - 1;
  ring_ = (Record *) heap_caps_malloc((size_t) ring_cap_ * sizeof(Record), MALLOC_CAP_SPIRAM);
  if (ring_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate %u-record PSRAM ring", ring_cap_);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "PSRAM ring: %u records (%u MB); staging: %u frames (internal)",
           ring_cap_, ring_mb_, stage_cap_);

  // pre-roll ring (bounded): preroll_ms at an assumed worst-case ~8000 fps, capped
  g_pre_cap = round_up_pow2(std::min<uint32_t>(preroll_ms_ * 8u, 65536u));
  g_pre = (Record *) heap_caps_malloc((size_t) g_pre_cap * sizeof(Record), MALLOC_CAP_SPIRAM);
  g_pre_head = g_pre_count = 0;

  // TWAI node (on-chip), listen-only. The node driver computes bit timing from
  // the bitrate; P4 is classic CAN only (no FD).
  // With no tx_pin the controller has no output at all (IDF allows tx = -1 only
  // in listen-only), so the logger cannot disturb the bus whichever way the
  // unit's yellow/white wires turn out to be; the transceiver's TXD pull-up
  // holds it recessive.
  twai_onchip_node_config_t ncfg = {};
  ncfg.io_cfg.tx = tx_pin_ >= 0 ? (gpio_num_t) tx_pin_ : GPIO_NUM_NC;
  ncfg.io_cfg.rx = (gpio_num_t) rx_pin_;  // CRX
  ncfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  ncfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  ncfg.bit_timing.bitrate = bitrate_;
  ncfg.tx_queue_depth = 0;  // listen-only: no TX queue
  ncfg.flags.enable_listen_only = 1;

  if (twai_new_node_onchip(&ncfg, &node_) != ESP_OK) {
    ESP_LOGE(TAG, "twai_new_node_onchip failed (tx=%d rx=%d)", tx_pin_, rx_pin_);
    this->mark_failed();
    return;
  }
  twai_event_callbacks_t cbs = {};
  cbs.on_rx_done = &CanLogger::on_rx_;
  cbs.on_error = &CanLogger::on_err_;
  if (twai_node_register_event_callbacks(node_, &cbs, this) != ESP_OK ||
      twai_node_enable(node_) != ESP_OK) {
    ESP_LOGE(TAG, "TWAI node enable failed");
    this->mark_failed();
    return;
  }

  epoch_us_ = 0;  // boot-relative; an ESPHome time: source can be added later

  // tasks: capture pinned to core 1 (dedicated, high prio); writer on core 0.
  xTaskCreatePinnedToCore([](void *a) { ((CanLogger *) a)->capture_task(); }, "can_cap",
                          4096, this, 20, nullptr, 1);
  xTaskCreatePinnedToCore([](void *a) { ((CanLogger *) a)->writer_task(); }, "can_wr",
                          8192, this, 5, nullptr, 0);

  this->start_capture();
}

// ---- capture task (core 1): staging ring -> PSRAM ring --------------------
void CanLogger::capture_task() {
  RawFrame rf;
  for (;;) {
    bool got = false;
    while (pop_stage_(rf)) {
      got = true;
      if (!running_.load())
        continue;  // disarmed: drain + discard so staging never backs up
      Record r;
      std::memset(&r, 0, sizeof(r));
      r.ts_us = rf.ts_us;
      r.id = rf.id;
      r.dlc = rf.dlc;
      if (gap_pending_.exchange(false))
        r.flags |= REC_FLAG_GAP;
      std::memcpy(r.data, rf.data, rf.dlc);
      if (!ring_push_(r)) {
        // PSRAM ring full (SD stalled too long) = also data loss
        overrun_total_.fetch_add(1);
        gap_pending_.store(true);
      } else {
        frames_total_.fetch_add(1);
        // SOF..EOF + 3-bit intermission: 47 bits standard, 67 extended, plus data
        uint32_t bits = (rf.id & (1u << 31)) ? 67 : 47;
        if (!(rf.id & (1u << 30)))
          bits += 8u * rf.dlc;
        bits_total_.fetch_add(bits);
      }
    }
    // fold any ISR-side staging overflow into the integrity gate
    uint32_t so = stage_overrun_.exchange(0);
    if (so)
      overrun_total_.fetch_add(so);
    if (!got)
      vTaskDelay(1);  // ~1 tick; staging absorbs the gap
  }
}

// ---- writer task (core 0) -------------------------------------------------
// FatFS only updates a file's size in its directory entry on f_sync/f_close,
// so without this an open capture reads back as 0 bytes over HTTP and a power
// cut loses everything since it was opened.
static void sync_file(FILE *f) {
  if (f == nullptr)
    return;
  fflush(f);
  if (fsync(fileno(f)) != 0)
    ESP_LOGW(TAG, "capture fsync failed");
}

void CanLogger::write_header_(void *fp) {
  FileHeader h;
  std::memset(&h, 0, sizeof(h));
  std::memcpy(h.magic, "CANBIN01", 8);
  h.version = 1;
  h.rec_size = sizeof(Record);
  h.bitrate = bitrate_;
  h.epoch_us = epoch_us_;
  fwrite(&h, sizeof(h), 1, (FILE *) fp);
}

void CanLogger::open_segment_() {
  static uint32_t seg_idx = UINT32_MAX;
  if (seg_idx == UINT32_MAX) {
    seg_idx = next_free_seg_index(sd_path_);
    g_boot_seg = seg_idx;
  }
  char path[160];
  snprintf(path, sizeof(path), "%s/seg%04u.canbin", sd_path_.c_str(), seg_idx++);
  g_seg_file = fopen(path, "wb");
  if (g_seg_file) {
    write_header_(g_seg_file);
    sync_file(g_seg_file);
    g_seg_bytes_written = sizeof(FileHeader);
    ESP_LOGI(TAG, "segment -> %s", path);
  } else {
    ESP_LOGE(TAG, "cannot open %s", path);
    sd_ok_.store(false);
  }
}

void CanLogger::close_segment_() {
  if (g_seg_file) {
    fclose(g_seg_file);
    g_seg_file = nullptr;
  }
}

void CanLogger::open_flash_file_(const Record &trigger) {
  char path[160];
  // boot's first segment index + ms since boot: unique across reboots
  snprintf(g_flash_name, sizeof(g_flash_name), "flash_%04u_%llu.canbin", (unsigned) g_boot_seg,
           (unsigned long long) (trigger.ts_us / 1000));
  snprintf(path, sizeof(path), "%s/%s", sd_path_.c_str(), g_flash_name);
  g_flash_file = fopen(path, "wb");
  if (!g_flash_file) {
    ESP_LOGE(TAG, "cannot open flash file %s", path);
    return;
  }
  write_header_(g_flash_file);
  g_flash_bytes_written = sizeof(FileHeader);

  // meta marker, then dump the pre-roll ring oldest->newest
  Record m;
  std::memset(&m, 0, sizeof(m));
  m.ts_us = trigger.ts_us;
  m.id = REC_META | (trigger.id & 0x1FFFFFFFu);
  m.dlc = EV_FLASH_START;
  fwrite(&m, sizeof(m), 1, g_flash_file);
  g_flash_bytes_written += sizeof(Record);

  // The ring is sized by frame count; only frames from the last preroll_ms go in,
  // or a quiet bus would drag in traffic from long before the session.
  const uint64_t window_us = (uint64_t) preroll_ms_ * 1000ull;
  const uint64_t oldest = trigger.ts_us > window_us ? trigger.ts_us - window_us : 0;
  uint32_t n = g_pre_count, kept = 0;
  uint32_t start = (g_pre_head + g_pre_cap - n) & (g_pre_cap - 1);
  for (uint32_t i = 0; i < n; i++) {
    Record &pr = g_pre[(start + i) & (g_pre_cap - 1)];
    if (pr.ts_us < oldest)
      continue;
    fwrite(&pr, sizeof(Record), 1, g_flash_file);
    g_flash_bytes_written += sizeof(Record);
    kept++;
  }
  sync_file(g_flash_file);
  flash_active_.store(true);
  ESP_LOGI(TAG, "FLASH session start -> %s (pre-roll %u recs)", path, kept);
}

void CanLogger::close_flash_file_() {
  if (!g_flash_file)
    return;
  Record m;
  std::memset(&m, 0, sizeof(m));
  m.ts_us = (uint64_t) esp_timer_get_time();
  m.id = REC_META;
  m.dlc = EV_FLASH_END;
  fwrite(&m, sizeof(m), 1, g_flash_file);
  g_flash_bytes_written += sizeof(Record);
  fclose(g_flash_file);
  g_flash_file = nullptr;
  flash_active_.store(false);
  last_flash_bytes_.store(g_flash_bytes_written);
  std::strncpy(g_last_flash_name, g_flash_name, sizeof(g_last_flash_name) - 1);
  g_flash_end_deadline = 0;
  ESP_LOGI(TAG, "FLASH session end: %s (%u bytes)", g_flash_name, g_flash_bytes_written);
}

// Inspect one record for UDS flash boundaries, maintain pre-roll + files.
void CanLogger::writer_handle_record_(const Record &r) {
  const bool meta = (r.id & REC_META) != 0;

  // UDS service detection on ISO-TP SF/FF (sniffed, tester->ECU direction)
  if (!meta && r.dlc >= 2 && (diag_tx_id_ == 0 || (r.id & 0x1FFFFFFFu) == diag_tx_id_)) {
    uint8_t pci = r.data[0] >> 4;
    uint8_t svc = 0xFF;
    if (pci == 0x0 && r.dlc >= 2) svc = r.data[1];        // single frame
    else if (pci == 0x1 && r.dlc >= 3) svc = r.data[2];   // first frame
    if (svc == 0x34 && !g_flash_file) {
      open_flash_file_(r);
    } else if (svc == 0x37 && g_flash_file) {
      // begin post-roll: keep writing until deadline
      g_flash_end_deadline = r.ts_us + (uint64_t) postroll_ms_ * 1000ull;
    }
  }

  // write to free-run segment
  if (g_seg_file) {
    fwrite(&r, sizeof(Record), 1, g_seg_file);
    g_seg_bytes_written += sizeof(Record);
    bytes_written_.fetch_add(sizeof(Record));
    if (g_seg_bytes_written >= seg_bytes_) {
      close_segment_();
      open_segment_();
    }
  }

  // write to flash file if active
  if (g_flash_file) {
    fwrite(&r, sizeof(Record), 1, g_flash_file);
    g_flash_bytes_written += sizeof(Record);
    if (g_flash_end_deadline && r.ts_us >= g_flash_end_deadline)
      close_flash_file_();
  }

  // feed the pre-roll ring last (data frames only): fed first, the 0x34 trigger
  // would be dumped with the pre-roll and then written again above
  if (!meta && g_pre) {
    g_pre[g_pre_head] = r;
    g_pre_head = (g_pre_head + 1) & (g_pre_cap - 1);
    if (g_pre_count < g_pre_cap)
      g_pre_count++;
  }
}

// SDMMC slot 0, 4-bit, FAT. Mirrors M5's Tab5 BSP (bsp_sdcard_init): the slot-0
// IO pins are powered from on-chip LDO channel 4. Never formats the card.
bool CanLogger::mount_sd_() {
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.slot = SDMMC_HOST_SLOT_0;
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
  if (sd_ldo_chan_ > 0) {
    if (sd_pwr_ == nullptr) {
      sd_pwr_ctrl_ldo_config_t ldo = {};
      ldo.ldo_chan_id = sd_ldo_chan_;
      sd_pwr_ctrl_handle_t h = nullptr;
      if (sd_pwr_ctrl_new_on_chip_ldo(&ldo, &h) != ESP_OK) {
        ESP_LOGE(TAG, "SD: on-chip LDO %d power control failed", sd_ldo_chan_);
        return false;
      }
      sd_pwr_ = h;
    }
    host.pwr_ctrl_handle = (sd_pwr_ctrl_handle_t) sd_pwr_;
  }

  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 4;
  slot.clk = (gpio_num_t) sd_clk_;
  slot.cmd = (gpio_num_t) sd_cmd_;
  slot.d0 = (gpio_num_t) sd_d0_;
  slot.d1 = (gpio_num_t) sd_d1_;
  slot.d2 = (gpio_num_t) sd_d2_;
  slot.d3 = (gpio_num_t) sd_d3_;

  esp_vfs_fat_sdmmc_mount_config_t mc = {};
  mc.format_if_mount_failed = false;
  mc.max_files = 6;
  mc.allocation_unit_size = 16 * 1024;

  sdmmc_card_t *card = nullptr;
  esp_err_t err = esp_vfs_fat_sdmmc_mount(sd_path_.c_str(), &host, &slot, &mc, &card);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "SD mount at %s failed: %s (card inserted, FAT32/exFAT?)", sd_path_.c_str(),
             esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "SD mounted at %s: %s, %llu MB", sd_path_.c_str(), card->cid.name,
           (unsigned long long) card->csd.capacity * card->csd.sector_size / (1024 * 1024));
  return true;
}

// Files are closed but the card is left mounted: closed files plus no writes in
// flight is what makes power-off safe, and unmounting would deinit the SDMMC
// host slot that the C6 Wi-Fi link shares.
void CanLogger::finish_shutdown_() {
  running_.store(false);
  vTaskDelay(pdMS_TO_TICKS(20));  // let an in-flight ring push land
  Record r;
  uint32_t drained = 0;
  while (ring_pop_(r)) {
    writer_handle_record_(r);
    drained++;
  }
  close_flash_file_();  // writes the FLASH_END marker if a session was open
  close_segment_();
  shutdown_done_.store(true);
  ESP_LOGI(TAG, "safe shutdown: %u buffered records written, files closed - safe to power off",
           (unsigned) drained);
}

void CanLogger::writer_task() {
  // capture keeps running into the PSRAM ring while we wait for a card
  while (!mount_sd_()) {
    if (shutdown_req_.load()) {  // nothing open: already safe
      running_.store(false);
      shutdown_done_.store(true);
      for (;;)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  sd_ok_.store(true);
  open_segment_();

  Record r;
  int64_t last_sync = esp_timer_get_time();
  for (;;) {
    if (shutdown_req_.load()) {
      finish_shutdown_();
      for (;;)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    bool got = false;
    while (ring_pop_(r)) {
      writer_handle_record_(r);
      got = true;
    }
    // once a second: make the data durable and visible to the HTTP reader
    int64_t now = esp_timer_get_time();
    if (now - last_sync >= 1000000) {
      sync_file(g_seg_file);
      sync_file(g_flash_file);
      last_sync = now;
    }
    if (got) {
      if (g_seg_file) fflush(g_seg_file);
      if (g_flash_file) fflush(g_flash_file);
    } else {
      // if in post-roll with no traffic, honor the deadline by wall clock
      if (g_flash_file && g_flash_end_deadline &&
          (uint64_t) esp_timer_get_time() >= g_flash_end_deadline)
        close_flash_file_();
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

void CanLogger::start_capture() {
  if (shutdown_req_.load())
    return;  // files are closed; only a reboot resumes
  if (running_.exchange(true))
    return;
  ESP_LOGI(TAG, "capture armed");
}
void CanLogger::stop_capture() { running_.store(false); }

std::string CanLogger::last_flash_file() const { return std::string(g_last_flash_name); }

// ---- HTTP file server -----------------------------------------------------
static esp_err_t http_index(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_sendstr_chunk(req, "<html><body><p><a href=\"/\">Device page</a></p>"
                                "<h3>Tab5 CAN captures</h3><ul>");
  DIR *d = opendir(g_sd_path.c_str());
  if (d) {
    struct dirent *e;
    char line[256];
    while ((e = readdir(d))) {
      if (std::strstr(e->d_name, ".canbin")) {
        snprintf(line, sizeof(line), "<li><a href=\"/dl?f=%s\">%s</a></li>", e->d_name,
                 e->d_name);
        httpd_resp_sendstr_chunk(req, line);
      }
    }
    closedir(d);
  }
  httpd_resp_sendstr_chunk(req, "</ul></body></html>");
  httpd_resp_sendstr_chunk(req, nullptr);
  return ESP_OK;
}

static esp_err_t http_dl(httpd_req_t *req) {
  char q[128] = {0}, fn[96] = {0};
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "f", fn, sizeof(fn)) != ESP_OK ||
      std::strstr(fn, "..")) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad f");
    return ESP_FAIL;
  }
  char path[200];
  snprintf(path, sizeof(path), "%s/%s", g_sd_path.c_str(), fn);
  FILE *f = fopen(path, "rb");
  if (!f) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no file");
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "application/octet-stream");
  static char buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
    httpd_resp_send_chunk(req, buf, n);
  fclose(f);
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}

#ifdef USE_CAN_LOGGER_WEB
bool CanFileHandler::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET)
    return false;
  char buf[AsyncWebServerRequest::URL_BUF_SIZE];
  StringRef url = request->url_to(buf);
  return url == "/files" || url == "/dl";
}

void CanFileHandler::handleRequest(AsyncWebServerRequest *request) {
  // web_server_idf wraps the raw httpd request; stream it the same way the
  // standalone server does (blocking chunked sends in the httpd task).
  httpd_req_t *req = *request;
  char buf[AsyncWebServerRequest::URL_BUF_SIZE];
  if (request->url_to(buf) == "/dl")
    http_dl(req);
  else
    http_index(req);
}
#endif

// ---- loop / dump ----------------------------------------------------------
void CanLogger::loop() {
  // frame rate and bus load over ~1 s windows; the capture task only adds counters
  int64_t now = esp_timer_get_time();
  if (rate_t_ == 0 || now - rate_t_ >= 1000000) {
    uint64_t f = frames_total_.load(), b = bits_total_.load();
    if (rate_t_ != 0) {
      float dt = (now - rate_t_) / 1e6f;
      frame_rate_ = (f - rate_frames_) / dt;
      bus_load_ = 100.0f * (b - rate_bits_) / dt / bitrate_;
      if (frame_rate_ > peak_rate_)
        peak_rate_ = frame_rate_;
    }
    rate_t_ = now;
    rate_frames_ = f;
    rate_bits_ = b;
  }

  // start the standalone HTTP server once, after the C6/WiFi link is up
  if (!use_web_ && g_httpd == nullptr && network_is_connected_hint_()) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = http_port_;
    cfg.lru_purge_enable = true;
    if (httpd_start(&g_httpd, &cfg) == ESP_OK) {
      httpd_uri_t i = {"/", HTTP_GET, http_index, nullptr};
      httpd_uri_t l = {"/files", HTTP_GET, http_index, nullptr};
      httpd_uri_t d = {"/dl", HTTP_GET, http_dl, nullptr};
      httpd_register_uri_handler(g_httpd, &i);
      httpd_register_uri_handler(g_httpd, &l);
      httpd_register_uri_handler(g_httpd, &d);
      ESP_LOGI(TAG, "file server on :%u", http_port_);
    }
  }
}

// We don't have ESPHome's WiFi object here; a tiny delay-based gate is enough
// since setup_priority::LATE runs after network. Replace with a proper check
// if you wire the wifi component in. For now: start after ~3 s uptime.
bool CanLogger::network_is_connected_hint_() {
  return esp_timer_get_time() > 3'000'000;
}

void CanLogger::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 CAN logger (esp_twai node API, listen-only):");
  ESP_LOGCONFIG(TAG, "  tx=%d%s rx=%d  bitrate=%u (classic CAN, no FD)", tx_pin_,
                tx_pin_ < 0 ? " (none: cannot drive bus)" : "", rx_pin_, bitrate_);
  ESP_LOGCONFIG(TAG, "  SD slot0 clk=%d cmd=%d d0-3=%d/%d/%d/%d ldo=%d", sd_clk_, sd_cmd_,
                sd_d0_, sd_d1_, sd_d2_, sd_d3_, sd_ldo_chan_);
  ESP_LOGCONFIG(TAG, "  sd=%s  ring=%uMB  seg=%uB  staging=%u frames", sd_path_.c_str(),
                ring_mb_, seg_bytes_, stage_cap_);
  ESP_LOGCONFIG(TAG, "  diag_tx_id=0x%X  preroll=%ums postroll=%ums", diag_tx_id_,
                preroll_ms_, postroll_ms_);
  ESP_LOGCONFIG(TAG, "  overruns=%llu bus_errors=%llu",
                (unsigned long long) overrun_total_.load(),
                (unsigned long long) bus_errors_.load());
}

}  // namespace can_logger
}  // namespace esphome
