#pragma once
// Second, CAN FD channel: a gs_usb adapter (CANable 2.0, STM32G431) with
// Elmue's CANable 2.5 firmware on the Tab5's USB-A host port. Built only with
// `fd_channel:` in the YAML (USE_CAN_LOGGER_GSUSB).
//
//   USB lib task (core 1, prio 19): usb_host_lib_handle_events (enumeration).
//   USB client task (core 1, prio 18): opens the adapter, sets it listen-only
//       (bus monitoring) with FD bit timing, timestamps and blob transfers, keeps
//       bulk IN transfers queued, parses them into an SPSC ring that the capture
//       task drains. Frames the adapter or this ring had to drop are counted.
//
// The USB stack runs in tasks, so while the flash cache is off it stops; the
// adapter's own queue (70 frames in Elmue's firmware) covers that time.

#include "esphome/core/defines.h"  // USE_CAN_LOGGER_GSUSB

#ifdef USE_CAN_LOGGER_GSUSB

#include <atomic>
#include <cstdint>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

namespace esphome {
namespace can_logger {

// One frame from the adapter, staged for the capture task.
struct FdFrame {
  uint64_t ts_us;    // esp_timer time (the adapter's 1 us stamp, mapped)
  uint32_t id;       // bit31 = extended, bit30 = RTR, bits0-28 = id
  uint8_t len;       // data bytes 0..64 (RTR: the requested DLC)
  uint8_t flags;     // FD_F_* below
  uint8_t data[64];
};

static const uint8_t FD_F_FDF = 1u << 0;  // CAN FD frame
static const uint8_t FD_F_BRS = 1u << 1;  // bit rate switch
static const uint8_t FD_F_ESI = 1u << 2;  // sender error passive
static const uint8_t FD_F_GAP = 1u << 7;  // frames were lost before this one

class GsUsbFd {
 public:
  void set_bitrates(uint32_t nominal, uint32_t data) { nominal_ = nominal; data_ = data; }
  void set_sample_points(float nominal, float data) { sp_nom_ = nominal; sp_data_ = data; }
  // Desk test: adapter in internal loopback (never drives the bus), the client
  // task sends this many numbered frames and they come back as received ones.
  void set_test_frames(uint32_t n) { test_frames_ = n; }

  bool start();               // allocates the ring, starts the USB tasks
  bool pop(FdFrame &out);     // capture task (single consumer)

  // stats (any task)
  uint32_t loss_events() const { return loss_events_.load(); }  // adapter or ring dropped frames
  uint32_t staged_peak() const { return ring_peak_.load(); }
  uint32_t ring_size() const { return ring_cap_; }
  float bus_load_pct() const;  // the adapter's own measurement, 0 if not reported lately
  const char *state() const;   // "no adapter", "starting", "active", "warning", ...
  std::string adapter() const; // board name and firmware, once opened
  uint32_t data_bitrate() const { return data_; }
  uint32_t nominal_bitrate() const { return nominal_; }
  bool test_mode() const { return test_frames_ > 0; }

  // task entry points (public so FreeRTOS can reach them)
  void lib_task();
  void client_task();

 protected:
  enum State : uint8_t { ST_NONE, ST_OPENING, ST_RUNNING, ST_FAILED, ST_GONE };

  static void client_event_cb_(const usb_host_client_event_msg_t *msg, void *arg);
  static void ctrl_cb_(usb_transfer_t *xfer);
  static void in_cb_(usb_transfer_t *xfer);
  static void out_cb_(usb_transfer_t *xfer);

  bool open_device_(uint8_t addr);
  bool configure_();
  void close_device_();
  bool leave_dfu_();
  bool ctrl_raw_(uint8_t type, uint8_t request, uint16_t value, uint16_t index, void *data,
                 uint16_t len, uint32_t timeout_ms = 1000);
  bool ctrl_(uint8_t request, bool in, void *data, uint16_t len);
  void submit_in_(usb_transfer_t *xfer);
  void parse_(const uint8_t *p, int n, int64_t now);
  void handle_msg_(const uint8_t *m, uint8_t size, int64_t now);
  void rx_frame_(const uint8_t *m, uint8_t size, int64_t now);
  uint64_t map_ts_(uint32_t dev_ts, int64_t now);
  void push_(const FdFrame &f);
  void test_send_();

  // config
  uint32_t nominal_{500000}, data_{2000000};
  float sp_nom_{0.8f}, sp_data_{0.75f};
  uint32_t test_frames_{0};

  // USB
  TaskHandle_t client_th_{nullptr};
  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t dev_{nullptr};
  volatile int new_addr_{-1};       // set by the client event callback
  volatile bool dev_gone_{false};
  bool close_pending_{false};       // a close had to wait for transfers
  uint8_t ep_in_{0}, ep_out_{0}, iface_{0};
  bool iface_claimed_{false};
  usb_transfer_t *ctrl_xfer_{nullptr};
  volatile bool ctrl_done_{false};
  bool ctrl_inflight_{false};       // submitted, callback not yet run
  static constexpr int N_IN = 6;
  static constexpr int IN_BYTES = 2048;  // Elmue's MAX_BLOB_SIZE
  usb_transfer_t *in_xfer_[N_IN]{};
  usb_transfer_t *out_xfer_{nullptr};
  bool out_busy_{false};
  int inflight_{0};
  std::atomic<uint8_t> state_{ST_NONE};
  char adapter_[64]{};
  bool was_running_{false};         // a reconnect while capturing is a gap

  // SPSC ring (PSRAM): client task -> capture task
  FdFrame *ring_{nullptr};
  uint32_t ring_cap_{0}, ring_mask_{0};
  std::atomic<uint32_t> head_{0}, tail_{0};
  std::atomic<uint32_t> ring_peak_{0};
  std::atomic<uint32_t> loss_events_{0};
  bool gap_pending_{false};

  // adapter reports (client task writes, others read)
  std::atomic<uint8_t> bus_status_{0};  // 0 active, 1 warning, 2 passive, 3 bus-off
  std::atomic<uint8_t> busload_{0};
  std::atomic<int64_t> busload_t_{0};
  int64_t last_string_log_{0};

  // timestamp mapping: host = device (unwrapped) + offset; offset follows the
  // smallest host-minus-device difference seen in each 1 s window
  uint32_t last_dev_ts_{0};
  uint64_t dev_wrap_{0};
  bool have_offset_{false};
  int64_t offset_{0}, win_min_{INT64_MAX}, win_t_{0}, first_t_{0};
  uint64_t last_host_ts_{0};

  // loopback test
  uint32_t test_sent_{0}, test_rx_{0}, test_seq_err_{0}, test_next_rx_{0};
  int64_t test_t0_{0}, test_t1_{0};
  bool test_reported_{false};
};

}  // namespace can_logger
}  // namespace esphome

#endif  // USE_CAN_LOGGER_GSUSB
