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
#include <cmath>
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
uint32_t g_seg_bytes_written = 0;
httpd_handle_t g_httpd = nullptr;
std::string g_sd_path;            // copy for http handlers
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
  uint32_t used = (head + 1 - tail) & ring_mask_;
  if (used > ring_peak_.load(std::memory_order_relaxed))
    ring_peak_.store(used, std::memory_order_relaxed);  // only this task writes it
  return true;
}

bool CanLogger::ring_push_n_(const Record *r, uint32_t n) {
  uint32_t head = ring_head_.load(std::memory_order_relaxed);
  uint32_t tail = ring_tail_.load(std::memory_order_acquire);
  uint32_t used = (head - tail) & ring_mask_;
  if (used + n >= ring_cap_)
    return false;  // no room for the whole frame: caller counts it as loss
  for (uint32_t i = 0; i < n; i++)
    ring_[(head + i) & ring_mask_] = r[i];
  ring_head_.store(head + n, std::memory_order_release);
  if (used + n > ring_peak_.load(std::memory_order_relaxed))
    ring_peak_.store(used + n, std::memory_order_relaxed);
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

// Error state for the screen/HA. twai_node_get_info() can't be used for this:
// with a listen-only node and tx_queue_depth 0 there is no TX queue, and its
// status path asserts on uxQueueSpacesAvailable(NULL).
IRAM_ATTR bool CanLogger::on_state_(twai_node_handle_t node,
                                    const twai_state_change_event_data_t *edata, void *ctx) {
  ((CanLogger *) ctx)->bus_state_.store((uint8_t) edata->new_sta, std::memory_order_relaxed);
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

  // per-ID counters (capture task) in internal RAM; the loop's snapshot in PSRAM
  id_counts_ = (uint32_t *) heap_caps_calloc(2048, sizeof(uint32_t), MALLOC_CAP_INTERNAL);
  ext_ids_ = (uint32_t *) heap_caps_calloc(EXT_SLOTS, sizeof(uint32_t), MALLOC_CAP_INTERNAL);
  ext_counts_ = (uint32_t *) heap_caps_calloc(EXT_SLOTS, sizeof(uint32_t), MALLOC_CAP_INTERNAL);
  prev_counts_ = (uint32_t *) heap_caps_calloc(2048 + EXT_SLOTS, sizeof(uint32_t), MALLOC_CAP_SPIRAM);
  if (!id_counts_ || !ext_ids_ || !ext_counts_ || !prev_counts_)
    ESP_LOGW(TAG, "ID statistics disabled (no memory)");  // capture is unaffected

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
  cbs.on_state_change = &CanLogger::on_state_;
  if (twai_node_register_event_callbacks(node_, &cbs, this) != ESP_OK ||
      twai_node_enable(node_) != ESP_OK) {
    ESP_LOGE(TAG, "TWAI node enable failed");
    this->mark_failed();
    return;
  }
  bus_state_.store(TWAI_ERROR_ACTIVE);  // enable() leaves bus-off; no callback for it

  epoch_us_ = 0;  // boot-relative; an ESPHome time: source can be added later

  // tasks: capture pinned to core 1 (dedicated, high prio); writer on core 0.
  xTaskCreatePinnedToCore([](void *a) { ((CanLogger *) a)->capture_task(); }, "can_cap",
                          4096, this, 20, nullptr, 1);
  xTaskCreatePinnedToCore([](void *a) { ((CanLogger *) a)->writer_task(); }, "can_wr",
                          8192, this, 5, nullptr, 0);
#ifdef USE_CAN_LOGGER_GSUSB
  if (!gs_usb_.start())
    ESP_LOGE(TAG, "FD channel not started");  // channel 0 is unaffected
#endif

  this->start_capture();
}

#ifdef USE_CAN_LOGGER_GSUSB
// FD staging (PSRAM) -> PSRAM ring as bus-1 records: a head record, then the
// payload beyond 8 bytes in raw continuation records, pushed all or nothing.
void CanLogger::drain_fd_(bool &got) {
  FdFrame f;
  Record recs[4];
  while (gs_usb_.pop(f)) {
    got = true;
    if (!running_.load())
      continue;
    std::memset(recs, 0, sizeof(recs));
    Record &h = recs[0];
    h.ts_us = f.ts_us;
    h.id = f.id;
    h.dlc = f.len;
    h.bus = 1;
    h.flags = ((f.flags & FD_F_FDF) ? REC_FLAG_FDF : 0) | ((f.flags & FD_F_BRS) ? REC_FLAG_BRS : 0) |
              ((f.flags & FD_F_ESI) ? REC_FLAG_ESI : 0);
    if ((f.flags & FD_F_GAP) || fd_gap_pending_)
      h.flags |= REC_FLAG_GAP;
    uint8_t len = (f.id & (1u << 30)) ? 0 : f.len;  // RTR: no payload
    std::memcpy(h.data, f.data, len < 8 ? len : 8);
    uint32_t n = 1;
    if (len > 8) {
      h.cont = (uint8_t) ((len - 8 + sizeof(Record) - 1) / sizeof(Record));
      std::memcpy(&recs[1], f.data + 8, len - 8);
      n += h.cont;
    }
    if (!ring_push_n_(recs, n)) {
      fd_ring_overruns_.fetch_add(1);
      fd_gap_pending_ = true;
    } else {
      fd_gap_pending_ = false;
      fd_frames_total_.fetch_add(1);
    }
  }
}
#endif

// ---- capture task (core 1): staging ring -> PSRAM ring --------------------
void CanLogger::capture_task() {
  RawFrame rf;
  for (;;) {
    bool got = false;
    uint32_t staged = (stage_head_.load(std::memory_order_acquire) -
                       stage_tail_.load(std::memory_order_relaxed)) & stage_mask_;
    if (staged > stage_peak_.load(std::memory_order_relaxed))
      stage_peak_.store(staged, std::memory_order_relaxed);
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
        count_id_(rf.id);
      }
    }
#ifdef USE_CAN_LOGGER_GSUSB
    drain_fd_(got);
#endif
    // fold any ISR-side staging overflow into the integrity gate
    uint32_t so = stage_overrun_.exchange(0);
    if (so)
      overrun_total_.fetch_add(so);
    if (!got)
      vTaskDelay(1);  // ~1 tick; staging absorbs the gap
  }
}

// One array increment for a standard id; a short probe for an extended one.
// Extended ids beyond EXT_SLOTS distinct values are captured but not counted.
void CanLogger::count_id_(uint32_t id) {
  if (id_counts_ == nullptr)
    return;
  if (!(id & (1u << 31))) {
    id_counts_[id & 0x7FF]++;
    return;
  }
  uint32_t key = (id & 0x1FFFFFFFu) | (1u << 31);
  uint32_t h = (key * 2654435761u) >> 24;  // 8-bit hash for 256 slots
  for (uint32_t k = 0; k < 8; k++) {
    uint32_t s = (h + k) & (EXT_SLOTS - 1);
    if (ext_ids_[s] == key) {
      ext_counts_[s]++;
      return;
    }
    if (ext_ids_[s] == 0) {
      ext_counts_[s] = 1;
      ext_ids_[s] = key;
      return;
    }
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
#ifdef USE_CAN_LOGGER_GSUSB
  std::memcpy(h.magic, "CANBIN02", 8);  // bus-1 records and continuations
  h.version = 2;
  h.flags = gs_usb_.nominal_bitrate();
  h.reserved = gs_usb_.data_bitrate();
#endif
  fwrite(&h, sizeof(h), 1, (FILE *) fp);
}

void CanLogger::open_segment_() {
  static uint32_t seg_idx = UINT32_MAX;
  if (seg_idx == UINT32_MAX)
    seg_idx = next_free_seg_index(sd_path_);
  char path[160];
  seg_index_.store(seg_idx);
  snprintf(path, sizeof(path), "%s/seg%04u.canbin", sd_path_.c_str(), seg_idx++);
  g_seg_file = fopen(path, "wb");
  if (g_seg_file) {
    write_header_(g_seg_file);
    sync_file(g_seg_file);
    g_seg_bytes_written = sizeof(FileHeader);
    seg_cur_bytes_.store(g_seg_bytes_written);
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

// Every record goes to the current segment, which rotates at seg_bytes, but
// never between an FD frame's head record and its continuations.
void CanLogger::writer_handle_record_(const Record &r) {
  static uint8_t cont_left = 0;  // continuation records still to come
  if (cont_left)
    cont_left--;  // raw payload, not a record
  else
    cont_left = r.cont;
  if (g_seg_file == nullptr)
    return;
  fwrite(&r, sizeof(Record), 1, g_seg_file);
  g_seg_bytes_written += sizeof(Record);
  seg_cur_bytes_.store(g_seg_bytes_written);
  bytes_written_.fetch_add(sizeof(Record));
  if (g_seg_bytes_written >= seg_bytes_ && cont_left == 0) {
    close_segment_();
    open_segment_();
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
  // Free space once, here: f_getfree can scan the whole FAT, so it never runs
  // while capturing; sd_free_gb() subtracts what's been written since.
  uint64_t total = 0, free = 0;
  if (esp_vfs_fat_info(sd_path_.c_str(), &total, &free) == ESP_OK) {
    sd_total_mb_.store((uint32_t) (total >> 20));
    sd_free0_mb_.store((uint32_t) (free >> 20));
    ESP_LOGI(TAG, "SD: %u MB free of %u MB", sd_free0_mb_.load(), sd_total_mb_.load());
  }
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
      uint32_t took = (uint32_t) (esp_timer_get_time() - now);  // slow card = ring fills
      if (took > sd_sync_max_us_.load())
        sd_sync_max_us_.store(took);
      last_sync = now;
    }
    if (got) {
      if (g_seg_file) fflush(g_seg_file);
    } else {
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

// ---- details (ESPHome loop) -----------------------------------------------
// Every 2 s: distinct ids seen since boot and the busiest ids over the window.
// Reads the capture task's counters without locking: a count can be one frame
// stale, which doesn't matter for a rate.
void CanLogger::update_id_stats_() {
  int64_t now = esp_timer_get_time();
  if (id_counts_ == nullptr || prev_counts_ == nullptr) {
    ids_t_ = now;
    return;
  }
  float dt = ids_t_ ? (now - ids_t_) / 1e6f : 0;
  IdRate top[TOP_IDS] = {};
  uint8_t n = 0;
  uint32_t uniq = 0;
  auto consider = [&](uint32_t id, uint32_t count, uint32_t &prev) {
    uniq++;
    uint32_t d = count - prev;
    prev = count;
    if (dt <= 0 || d == 0)
      return;
    float rate = d / dt;
    uint8_t pos = n < TOP_IDS ? n : TOP_IDS;
    while (pos > 0 && top[pos - 1].rate < rate)
      pos--;
    if (pos >= TOP_IDS)
      return;
    for (uint8_t j = (n < TOP_IDS ? n : TOP_IDS - 1); j > pos; j--)
      top[j] = top[j - 1];
    top[pos] = {id, rate};
    if (n < TOP_IDS)
      n++;
  };
  for (uint32_t i = 0; i < 2048; i++)
    if (id_counts_[i])
      consider(i, id_counts_[i], prev_counts_[i]);
  for (uint32_t s = 0; s < EXT_SLOTS; s++)
    if (ext_ids_[s])
      consider(ext_ids_[s], ext_counts_[s], prev_counts_[2048 + s]);
  std::memcpy(top_, top, sizeof(top));
  top_n_ = n;
  unique_ids_ = uniq;
  ids_t_ = now;
}

std::string CanLogger::top_ids_text() const {
  std::string s;
  char b[32];
  for (uint8_t i = 0; i < top_n_; i++) {
    bool ext = top_[i].id & (1u << 31);
    snprintf(b, sizeof(b), ext ? "%s0x%08X %.0f/s" : "%s0x%03X %.0f/s", i ? ", " : "",
             (unsigned) (top_[i].id & 0x1FFFFFFFu), top_[i].rate);
    s += b;
  }
  return s.empty() ? "-" : s;
}

const char *CanLogger::bus_state() const {
  switch (bus_state_.load()) {
    case TWAI_ERROR_ACTIVE: return "active";
    case TWAI_ERROR_WARNING: return "warning";
    case TWAI_ERROR_PASSIVE: return "passive";
    case TWAI_ERROR_BUS_OFF: return "bus-off";
    default: return "?";
  }
}

float CanLogger::buffer_fill_pct() const {
  if (ring_cap_ == 0)
    return 0;
  uint32_t used = (ring_head_.load() - ring_tail_.load()) & ring_mask_;
  return 100.0f * used / ring_cap_;
}

float CanLogger::sd_free_gb() const {
  uint64_t written = bytes_written_.load();
  uint64_t free = (uint64_t) sd_free0_mb_.load() << 20;
  return written >= free ? 0 : (free - written) / 1073741824.0f;
}

#ifdef USE_CAN_LOGGER_GSUSB
bool CanLogger::fd_enabled() const { return true; }
float CanLogger::fd_bus_load_pct() const { return gs_usb_.bus_load_pct(); }
uint32_t CanLogger::fd_overruns() const { return gs_usb_.loss_events() + fd_ring_overruns_.load(); }
const char *CanLogger::fd_state() const { return gs_usb_.state(); }
std::string CanLogger::fd_adapter() const { return gs_usb_.adapter(); }
uint32_t CanLogger::fd_staging_peak() const { return gs_usb_.staged_peak(); }
#else
bool CanLogger::fd_enabled() const { return false; }
float CanLogger::fd_bus_load_pct() const { return 0; }
uint32_t CanLogger::fd_overruns() const { return 0; }
const char *CanLogger::fd_state() const { return "off"; }
std::string CanLogger::fd_adapter() const { return "-"; }
uint32_t CanLogger::fd_staging_peak() const { return 0; }
#endif

std::string CanLogger::segment_name() const {
  if (!sd_ok_.load())
    return "-";
  char b[24];
  snprintf(b, sizeof(b), "seg%04u.canbin", (unsigned) seg_index_.load());
  return b;
}

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
  httpd_resp_set_hdr(req, "Accept-Ranges", "bytes");
  // The response is chunked, so the size goes in its own header for the client
  // to check it got everything. Static: httpd keeps the pointers until it sends.
  static char size_hdr[24], content_range[64];
  struct stat st;
  long size = stat(path, &st) == 0 ? (long) st.st_size : 0;
  snprintf(size_hdr, sizeof(size_hdr), "%ld", size);
  httpd_resp_set_hdr(req, "X-File-Size", size_hdr);
  // "Range: bytes=N-" resumes a download that a Wi-Fi drop cut off
  char range[48] = {0};
  long start = 0;
  if (httpd_req_get_hdr_value_str(req, "Range", range, sizeof(range)) == ESP_OK &&
      sscanf(range, "bytes=%ld-", &start) == 1 && start > 0) {
    if (start >= size) {
      fclose(f);
      snprintf(content_range, sizeof(content_range), "bytes */%ld", size);
      httpd_resp_set_status(req, "416 Range Not Satisfiable");
      httpd_resp_set_hdr(req, "Content-Range", content_range);
      httpd_resp_send(req, nullptr, 0);
      return ESP_OK;
    }
    fseek(f, start, SEEK_SET);
    snprintf(content_range, sizeof(content_range), "bytes %ld-%ld/%ld", start, size - 1, size);
    httpd_resp_set_status(req, "206 Partial Content");
    httpd_resp_set_hdr(req, "Content-Range", content_range);
  }
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
    uint64_t f = frames_total_.load(), b = bits_total_.load(), fd = fd_frames_total_.load();
    if (rate_t_ != 0) {
      float dt = (now - rate_t_) / 1e6f;
      frame_rate_ = (f - rate_frames_) / dt;
      bus_load_ = 100.0f * (b - rate_bits_) / dt / bitrate_;
      if (frame_rate_ > peak_rate_)
        peak_rate_ = frame_rate_;
      fd_rate_ = (fd - fd_rate_frames_) / dt;
      if (fd_rate_ > fd_peak_rate_)
        fd_peak_rate_ = fd_rate_;
    }
    rate_t_ = now;
    rate_frames_ = f;
    rate_bits_ = b;
    fd_rate_frames_ = fd;
  }
  if (ids_t_ == 0 || now - ids_t_ >= 2000000)
    update_id_stats_();

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
  ESP_LOGCONFIG(TAG, "  overruns=%llu bus_errors=%llu",
                (unsigned long long) overrun_total_.load(),
                (unsigned long long) bus_errors_.load());
#ifdef USE_CAN_LOGGER_GSUSB
  ESP_LOGCONFIG(TAG, "  FD channel (bus 1): gs_usb on USB-A, %u/%u bit/s%s, state %s, adapter %s",
                (unsigned) gs_usb_.nominal_bitrate(), (unsigned) gs_usb_.data_bitrate(),
                gs_usb_.test_mode() ? " INTERNAL LOOPBACK TEST" : " listen-only", gs_usb_.state(),
                gs_usb_.adapter().c_str());
#endif
}

}  // namespace can_logger
}  // namespace esphome
