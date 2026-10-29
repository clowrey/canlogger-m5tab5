#pragma once
// Tab5 CAN logger — real-time capture engine for the M5Stack Tab5 (ESP32-P4)
// + Unit CAN (CA-IS3050G) transceiver.
//
// Design (see README.md):
//   on_rx_ (IRAM, cache-safe ISR, esp_twai node API): frame -> internal-RAM
//       staging ring; staging overflow counts as an overrun.
//   capture_task (core 1, prio 20): staging ring -> 24-byte Record -> PSRAM ring;
//       PSRAM-ring overflow counts as an overrun. Never gated by ESPHome's loop.
//   writer_task (core 0, prio 5): drains the PSRAM ring -> SD. Free-runs every
//       frame to a rotating segment file, and carves a separate flash session file
//       from UDS RequestDownload (0x34) to RequestTransferExit (0x37) + post-roll.
//   ESPHome loop(): rate/bus-load maths and stats for entities and the screen.

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <atomic>
#include <cstdint>

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#ifdef USE_CAN_LOGGER_WEB
#include "esphome/components/web_server_base/web_server_base.h"
#endif

namespace esphome {
namespace can_logger {

#ifdef USE_CAN_LOGGER_WEB
// Serves /files (capture list) and /dl?f= (download) on ESPHome's web server,
// behind its auth. web_server is what makes HA's device page show a Visit link.
class CanFileHandler : public AsyncWebHandler {
 public:
  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  bool isRequestHandlerTrivial() const override { return false; }
};
#endif

// ---- on-disk record format (.canbin), little-endian -----------------------
// 32-byte file header, then fixed 24-byte records. Keep in lockstep with
// scripts/canbin2candump.py and scripts/reassemble_uds.py.
#pragma pack(push, 1)
struct FileHeader {
  char magic[8];      // "CANBIN01"
  uint16_t version;   // 1
  uint16_t rec_size;  // 24
  uint32_t bitrate;   // e.g. 500000
  uint64_t epoch_us;  // wall epoch of ts=0, or 0 if boot-relative
  uint32_t flags;     // reserved
  uint32_t reserved;
};

struct Record {
  uint64_t ts_us;     // microseconds since capture epoch
  uint32_t id;        // bits0-28 = CAN id; bit31 = extended; bit30 = RTR; bit29 = meta
  uint8_t dlc;        // data frame: 0..8 ; meta record: event code (see EventCode)
  uint8_t flags;      // bit0 = gap_before (frames lost to overrun before this one)
  uint8_t bus;        // 0 = primary channel (room for a 2nd controller later)
  uint8_t reserved;
  uint8_t data[8];    // unused bytes zero
};
#pragma pack(pop)

static_assert(sizeof(FileHeader) == 32, "FileHeader must be 32 bytes");
static_assert(sizeof(Record) == 24, "Record must be 24 bytes");

static const uint32_t REC_META = 1u << 29;  // id bit marking a meta/event record
static const uint8_t REC_FLAG_GAP = 1u << 0;

enum EventCode : uint8_t {
  EV_CAPTURE_START = 1,  // reserved, not written yet
  EV_FLASH_START = 2,    // flash file only: UDS RequestDownload (0x34) seen; id = trigger CAN id
  EV_FLASH_END = 3,      // flash file only: post-roll ended, file closed
  EV_OVERRUN_GAP = 4,    // reserved, not written yet (loss is marked with REC_FLAG_GAP)
  EV_SEG_ROTATE = 5,     // reserved, not written yet
};

// Internal-RAM staging entry: produced by the IRAM RX ISR, consumed by the
// capture task. Must live in internal RAM (an IRAM/cache-safe ISR cannot touch
// PSRAM), so this is deliberately NOT the on-disk Record (which goes to PSRAM).
struct RawFrame {
  uint64_t ts_us;
  uint32_t id;      // bit31 = extended, bit30 = RTR, bits0-28 = id
  uint8_t dlc;
  uint8_t data[8];
};

class CanLogger : public Component {
 public:
  // --- config setters (called from __init__.py codegen) ---
  void set_tx_pin(int p) { tx_pin_ = p; }
  void set_rx_pin(int p) { rx_pin_ = p; }
  void set_bitrate(uint32_t b) { bitrate_ = b; }
  void set_sd_path(const std::string &p) { sd_path_ = p; }
  void set_ring_mb(uint32_t mb) { ring_mb_ = mb; }
  void set_diag_tx_id(uint32_t id) { diag_tx_id_ = id; }
  void set_preroll_ms(uint32_t ms) { preroll_ms_ = ms; }
  void set_postroll_ms(uint32_t ms) { postroll_ms_ = ms; }
  void set_seg_bytes(uint32_t b) { seg_bytes_ = b; }
  void set_http_port(uint16_t p) { http_port_ = p; }
  void set_sd_pins(int clk, int cmd, int d0, int d1, int d2, int d3) {
    sd_clk_ = clk; sd_cmd_ = cmd; sd_d0_ = d0; sd_d1_ = d1; sd_d2_ = d2; sd_d3_ = d3;
  }
  void set_sd_ldo_channel(int ch) { sd_ldo_chan_ = ch; }
#ifdef USE_CAN_LOGGER_WEB
  // Called from codegen, before web_server's setup adds its own handler.
  void set_web_server_base(web_server_base::WebServerBase *base) {
    base->add_handler(&file_handler_);
    use_web_ = true;
  }
#endif

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  // --- stats exposed to ESPHome template entities (thread-safe reads) ---
  uint64_t frames_total() const { return frames_total_.load(); }
  uint64_t overrun_total() const { return overrun_total_.load(); }
  uint64_t bytes_written() const { return bytes_written_.load(); }
  bool sd_ok() const { return sd_ok_.load(); }
  bool flash_active() const { return flash_active_.load(); }
  bool capture_valid() const { return overrun_total_.load() == 0; }  // integrity gate
  std::string last_flash_file() const;
  uint32_t last_flash_bytes() const { return last_flash_bytes_.load(); }
  uint32_t bus_errors() const { return bus_errors_.load(); }  // diagnostic only
  // Live rate over the last second, updated in loop() (read from ESPHome's loop only).
  // Bus load counts nominal frame bits without stuff bits, so it reads a little low.
  float frame_rate() const { return frame_rate_; }
  float peak_frame_rate() const { return peak_rate_; }
  float bus_load_pct() const { return bus_load_; }

  // --- details for entities and the screen (read from ESPHome's loop) ---
  struct IdRate {
    uint32_t id;  // bit31 = extended
    float rate;   // frames/s over the last ID window
  };
  static constexpr uint8_t TOP_IDS = 5;
  const char *bus_state() const;           // controller error state
  uint32_t unique_ids() const { return unique_ids_; }
  uint8_t top_id_count() const { return top_n_; }
  const IdRate &top_id(uint8_t i) const { return top_[i]; }
  std::string top_ids_text() const;        // "0x123 812/s, ..."
  float buffer_fill_pct() const;           // PSRAM ring now
  float buffer_peak_pct() const { return ring_cap_ ? 100.0f * ring_peak_.load() / ring_cap_ : 0; }
  uint32_t staging_peak() const { return stage_peak_.load(); }
  uint32_t staging_size() const { return stage_cap_; }
  float sd_total_gb() const { return sd_total_mb_.load() / 1024.0f; }
  float sd_free_gb() const;                // estimate: free at mount minus bytes written since
  float sd_sync_max_ms() const { return sd_sync_max_us_.load() / 1000.0f; }
  std::string segment_name() const;
  float segment_mb() const { return seg_cur_bytes_.load() / 1048576.0f; }
  // UDS flash progress, sniffed on diag_tx_id: the current download (0x34 .. 0x37)
  int uds_last_service() const { return uds_last_svc_.load(); }  // -1 = none yet
  uint32_t uds_download_addr() const { return uds_dl_addr_.load(); }
  uint32_t uds_download_size() const { return uds_dl_size_.load(); }  // from 0x34
  uint32_t uds_download_bytes() const { return uds_dl_bytes_.load(); }  // 0x36 data so far
  float uds_progress_pct() const;          // NAN until a 0x34 announced a size
  uint32_t uds_blocks() const { return uds_blocks_.load(); }
  float flash_session_s() const;           // this or the last flash session
  std::string uds_last_service_text() const;  // "0x36 TransferData", "-" if none

  // --- controls ---
  void start_capture();   // arm/begin free-run (also called from setup)
  void stop_capture();
  // Stop capturing, write out everything buffered, end and close the flash
  // session and segment files. Done in writer_task (it owns the files); poll
  // shutdown_done(). Capture resumes only after a reboot.
  void safe_shutdown() { shutdown_req_.store(true); }
  bool shutdown_requested() const { return shutdown_req_.load(); }
  bool shutdown_done() const { return shutdown_done_.load(); }

  // internal task entrypoints (public so FreeRTOS can reach them)
  void capture_task();
  void writer_task();

 protected:
  // ring helpers (single-producer capture_task / single-consumer writer_task)
  bool ring_push_(const Record &r);          // producer
  bool ring_pop_(Record &out);               // consumer
  void writer_handle_record_(const Record &r);
  void open_segment_();
  void close_segment_();
  void open_flash_file_(const Record &trigger);
  void close_flash_file_();
  void write_header_(void *f);
  bool network_is_connected_hint_();

  // ESP-IDF node-API event callbacks (ISR context). IRAM_ATTR on the
  // definitions so they stay callable with the flash cache disabled.
  static bool on_rx_(twai_node_handle_t node, const twai_rx_done_event_data_t *edata,
                     void *ctx);
  static bool on_err_(twai_node_handle_t node, const twai_error_event_data_t *edata,
                      void *ctx);
  static bool on_state_(twai_node_handle_t node, const twai_state_change_event_data_t *edata,
                        void *ctx);
  void count_id_(uint32_t id);     // capture task: per-ID frame counters
  void update_id_stats_();         // ESPHome loop: unique ids + busiest ids
  void uds_sniff_(const Record &r);  // writer task: progress of the current download
  bool pop_stage_(RawFrame &out);  // consumer (capture task)
  bool mount_sd_();                // SDMMC slot 0 + FAT, called from writer_task
  void finish_shutdown_();         // writer_task side of safe_shutdown()

  // config
  int tx_pin_{-1}, rx_pin_{-1};
  uint32_t bitrate_{500000};
  std::string sd_path_{"/sdcard"};
  uint32_t ring_mb_{16};
  uint32_t diag_tx_id_{0};      // 0 = match UDS service on any id
  uint32_t preroll_ms_{5000};
  uint32_t postroll_ms_{2000};
  uint32_t seg_bytes_{64u * 1024 * 1024};
  uint16_t http_port_{80};
  bool use_web_{false};            // files served by ESPHome's web server instead
#ifdef USE_CAN_LOGGER_WEB
  CanFileHandler file_handler_;
#endif
  // microSD (Tab5 defaults: SDMMC slot 0, 4-bit, IO powered by on-chip LDO 4)
  int sd_clk_{43}, sd_cmd_{44}, sd_d0_{39}, sd_d1_{40}, sd_d2_{41}, sd_d3_{42};
  int sd_ldo_chan_{4};             // 0 = no on-chip LDO power control
  void *sd_pwr_{nullptr};          // sd_pwr_ctrl_handle_t

  // ESP-IDF TWAI node (one on-chip controller; P4 has three)
  twai_node_handle_t node_{nullptr};

  // staging ring (INTERNAL RAM): SPSC, producer = RX ISR, consumer = capture task
  RawFrame *stage_{nullptr};
  uint32_t stage_cap_{0};
  uint32_t stage_mask_{0};
  std::atomic<uint32_t> stage_head_{0};   // written by ISR
  std::atomic<uint32_t> stage_tail_{0};   // written by capture task
  std::atomic<uint32_t> stage_overrun_{0};  // ISR could not stage (frames lost)
  // 32-bit on purpose: RV32 has no native 64-bit atomics, so a uint64_t
  // fetch_add would call libatomic (flash) from the cache-safe ISR.
  std::atomic<uint32_t> bus_errors_{0};     // on_err_ events (diagnostic)

  // ring (PSRAM)
  Record *ring_{nullptr};
  uint32_t ring_cap_{0};        // record capacity (power of two)
  uint32_t ring_mask_{0};
  std::atomic<uint32_t> ring_head_{0};  // written by producer
  std::atomic<uint32_t> ring_tail_{0};  // written by consumer

  // stats
  std::atomic<uint64_t> frames_total_{0};
  std::atomic<uint64_t> overrun_total_{0};
  std::atomic<uint64_t> bytes_written_{0};
  std::atomic<bool> sd_ok_{false};
  std::atomic<bool> flash_active_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> shutdown_req_{false};
  std::atomic<bool> shutdown_done_{false};
  std::atomic<uint32_t> last_flash_bytes_{0};
  std::atomic<uint64_t> bits_total_{0};  // nominal wire bits of captured frames

  // rate window (ESPHome loop only)
  int64_t rate_t_{0};
  uint64_t rate_frames_{0}, rate_bits_{0};
  float frame_rate_{0}, peak_rate_{0}, bus_load_{0};

  // bus state (on_state_ ISR), buffer high-water marks (capture task)
  std::atomic<uint8_t> bus_state_{0};
  std::atomic<uint32_t> ring_peak_{0}, stage_peak_{0};

  // per-ID counters, written by the capture task only (plain 32-bit stores,
  // read approximately by the loop): standard ids direct, extended ids in a
  // small open-addressing table
  static constexpr uint32_t EXT_SLOTS = 256;
  uint32_t *id_counts_{nullptr};   // [2048] internal RAM
  uint32_t *ext_ids_{nullptr};     // [EXT_SLOTS] id | bit31, 0 = empty
  uint32_t *ext_counts_{nullptr};  // [EXT_SLOTS]
  // ID window (ESPHome loop only)
  uint32_t *prev_counts_{nullptr};  // [2048 + EXT_SLOTS] PSRAM
  int64_t ids_t_{0};
  uint32_t unique_ids_{0};
  IdRate top_[TOP_IDS]{};
  uint8_t top_n_{0};

  // SD (writer task)
  std::atomic<uint32_t> sd_total_mb_{0}, sd_free0_mb_{0}, sd_sync_max_us_{0};
  std::atomic<uint32_t> seg_index_{0}, seg_cur_bytes_{0};
  std::atomic<uint64_t> flash_written_{0};  // bytes into flash files since boot

  // UDS sniffing (writer task)
  std::atomic<int32_t> uds_last_svc_{-1};
  std::atomic<uint32_t> uds_dl_addr_{0}, uds_dl_size_{0}, uds_dl_bytes_{0}, uds_blocks_{0};
  std::atomic<uint32_t> sess_start_ms_{0}, sess_end_ms_{0};  // 0 = none / still open
  uint8_t req34_[32]{};            // 0x34 request being reassembled (ISO-TP)
  uint8_t req34_len_{0}, req34_have_{0};

  // producer-side pending gap flag (set when frames were lost)
  std::atomic<bool> gap_pending_{false};
  std::atomic<uint32_t> gap_lost_est_{0};

  uint64_t epoch_us_{0};
};

}  // namespace can_logger
}  // namespace esphome
