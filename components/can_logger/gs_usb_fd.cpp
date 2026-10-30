#include "gs_usb_fd.h"

#ifdef USE_CAN_LOGGER_GSUSB

#include "esphome/core/log.h"

#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace can_logger {

static const char *const TAG = "can_logger.fd";

// ---- Elmue's CANable 2.5 protocol (Firmware/Candlelight/candlelight_def.h) ----
// Control requests: vendor, recipient interface, wValue = channel, wIndex = interface.
enum : uint8_t {
  REQ_SET_BIT_TIMING = 1,
  REQ_SET_DEVICE_MODE = 2,
  REQ_GET_CAPABILITIES = 4,
  REQ_GET_DEVICE_VERSION = 5,
  REQ_SET_BIT_TIMING_FD = 10,
  REQ_GET_CAPABILITIES_FD = 11,
  REQ_GET_BOARD_INFO = 20,
  REQ_GET_LAST_ERROR = 22,      // 1 byte feedback for the previous request
  REQ_SET_BUSLOAD_REPORT = 23,  // interval in 100 ms units
};
static const uint8_t FBK_SUCCESS = 2;

// device flags (GS_ReqSetDeviceMode / capabilities)
static const uint32_t DEV_LISTEN_ONLY = 0x00001;
static const uint32_t DEV_LOOPBACK = 0x00002;    // with LISTEN_ONLY: internal loopback
static const uint32_t DEV_TIMESTAMP = 0x00010;
static const uint32_t DEV_CAN_FD = 0x00100;
static const uint32_t DEV_BIT_TIMING_FD = 0x00400;
static const uint32_t DEV_PROTOCOL_ELMUE = 0x04000;
static const uint32_t DEV_SEND_USB_BLOBS = 0x08000;

// messages on the bulk endpoints
enum : uint8_t {
  MSG_TX_FRAME = 0x0A,
  MSG_TX_ECHO = 0x0B,
  MSG_RX_FRAME = 0x0C,
  MSG_ERROR = 0x0D,
  MSG_STRING = 0x0E,
  MSG_BUSLOAD = 0x0F,
  MSG_TX_BLOB = 0x10,
  MSG_RX_BLOB = 0x11,
};
// frame flags
static const uint8_t FRM_FDF = 0x02, FRM_BRS = 0x04, FRM_ESI = 0x08;
// can_id flags
static const uint32_t ID_ERROR = 0x20000000, ID_RTR = 0x40000000, ID_EXT = 0x80000000;
// error report: err_id bits, data[1] bits, data[5] = firmware (app) flags
static const uint32_t ERID_BUS_OFF = 0x0040;
static const uint8_t ER1_RX_WARN = 0x04, ER1_TX_WARN = 0x08, ER1_RX_PASSIVE = 0x10,
                     ER1_TX_PASSIVE = 0x20;
static const uint8_t APP_CAN_RX_FAIL = 0x01, APP_USB_IN_OVERFLOW = 0x08;

static const uint16_t CANABLE_VID = 0x1D50, CANABLE_PID = 0x606F;
static const uint16_t STM32_DFU_VID = 0x0483, STM32_DFU_PID = 0xDF11;
static const uint32_t RING_FRAMES = 2048;  // ~160 KB PSRAM, ~0.25 s at the worst rate
static const uint16_t TEST_ID = 0x1FD;     // loopback test frames (std id, and ext 0x1FD1FD0)

#pragma pack(push, 1)
struct TimeLimits {
  uint32_t seg1_min, seg1_max, seg2_min, seg2_max, sjw_max, brp_min, brp_max, brp_inc;
};
struct CapabilityFD {
  uint32_t feature, fclk_can;
  TimeLimits nom, data;
};
struct BitTiming {
  uint32_t prop, seg1, seg2, sjw, brp;
};
struct DeviceMode {
  uint32_t mode, flags;
};
struct DeviceVersion {
  uint8_t hal_high, hal_mid, hal_low, icount;
  uint32_t sw_bcd, hw_bcd;
};
struct BoardInfo {
  uint16_t mcu_id;
  char mcu[25], board[25];
  uint32_t flags;
};
#pragma pack(pop)

static uint32_t rd32(const uint8_t *p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24);
}
static void wr32(uint8_t *p, uint32_t v) {
  p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

// FD data length codes
static const uint8_t FD_LENS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Bit timing for fclk: an exact prescaler, sample point closest to sp, the
// smallest prescaler on a tie (most time quanta).
static bool calc_timing(uint32_t fclk, uint32_t bitrate, float sp, const TimeLimits &t,
                        BitTiming &out) {
  float best = 1e9f;
  for (uint32_t brp = t.brp_min ? t.brp_min : 1; brp <= t.brp_max && brp <= 1024; brp++) {
    if (fclk % (brp * bitrate))
      continue;
    uint32_t tq = fclk / (brp * bitrate);
    if (tq < 1 + t.seg1_min + t.seg2_min || tq > 1 + t.seg1_max + t.seg2_max)
      continue;
    int32_t seg2 = (int32_t) tq - (int32_t) lroundf(tq * sp);
    if (seg2 < (int32_t) t.seg2_min) seg2 = t.seg2_min;
    if (seg2 > (int32_t) t.seg2_max) seg2 = t.seg2_max;
    int32_t seg1 = (int32_t) tq - 1 - seg2;
    if (seg1 < (int32_t) t.seg1_min || seg1 > (int32_t) t.seg1_max)
      continue;
    float err = fabsf((1.0f + seg1) / tq - sp);
    if (err < best - 1e-6f) {
      best = err;
      out.prop = 0;
      out.seg1 = seg1;
      out.seg2 = seg2;
      out.sjw = std::min<uint32_t>(std::min(seg1, seg2), t.sjw_max ? t.sjw_max : seg2);
      out.brp = brp;
    }
  }
  return best < 1e8f;
}

// ---- ring (client task -> capture task) ---------------------------------------
void GsUsbFd::push_(const FdFrame &f) {
  uint32_t head = head_.load(std::memory_order_relaxed);
  uint32_t tail = tail_.load(std::memory_order_acquire);
  if (head - tail >= ring_cap_) {
    loss_events_.fetch_add(1);
    gap_pending_ = true;
    return;
  }
  FdFrame &slot = ring_[head & ring_mask_];
  slot = f;
  if (gap_pending_) {
    slot.flags |= FD_F_GAP;
    gap_pending_ = false;
  }
  head_.store(head + 1, std::memory_order_release);
  uint32_t used = head + 1 - tail;
  if (used > ring_peak_.load(std::memory_order_relaxed))
    ring_peak_.store(used, std::memory_order_relaxed);
}

bool GsUsbFd::pop(FdFrame &out) {
  uint32_t tail = tail_.load(std::memory_order_relaxed);
  if (tail == head_.load(std::memory_order_acquire))
    return false;
  out = ring_[tail & ring_mask_];
  tail_.store(tail + 1, std::memory_order_release);
  return true;
}

// ---- start / tasks ------------------------------------------------------------
bool GsUsbFd::start() {
  ring_cap_ = RING_FRAMES;
  ring_mask_ = ring_cap_ - 1;
  ring_ = (FdFrame *) heap_caps_malloc(ring_cap_ * sizeof(FdFrame), MALLOC_CAP_SPIRAM);
  if (ring_ == nullptr) {
    ESP_LOGE(TAG, "FD ring alloc failed");
    return false;
  }
  // Client first: the lib task notifies it once the host library is installed.
  // The lib task must outrank the client: it hands a finished control transfer
  // to the client and only then lowers the device's in-flight count, so a client
  // that preempts it there and closes the device trips an assert in usbh_dev_close.
  xTaskCreatePinnedToCore([](void *a) { ((GsUsbFd *) a)->client_task(); }, "usb_fd", 6144, this,
                          18, &client_th_, 1);
  xTaskCreatePinnedToCore([](void *a) { ((GsUsbFd *) a)->lib_task(); }, "usb_lib", 4096, this, 19,
                          nullptr, 1);
  return true;
}

void GsUsbFd::lib_task() {
  usb_host_config_t cfg = {};
  cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
  esp_err_t err = usb_host_install(&cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(err));
    state_.store(ST_FAILED);
    vTaskDelete(nullptr);
    return;
  }
  xTaskNotifyGive(client_th_);
  for (;;) {
    uint32_t flags = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
  }
}

void GsUsbFd::client_event_cb_(const usb_host_client_event_msg_t *msg, void *arg) {
  GsUsbFd *self = (GsUsbFd *) arg;
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV)
    self->new_addr_ = msg->new_dev.address;
  else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE)
    self->dev_gone_ = true;
}

void GsUsbFd::client_task() {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // host library installed
  usb_host_client_config_t cc = {};
  cc.is_synchronous = false;
  cc.max_num_event_msg = 5;
  cc.async.client_event_callback = client_event_cb_;
  cc.async.callback_arg = this;
  if (usb_host_client_register(&cc, &client_) != ESP_OK ||
      usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + 128, 0, &ctrl_xfer_) != ESP_OK ||
      usb_host_transfer_alloc(IN_BYTES, 0, &out_xfer_) != ESP_OK) {
    ESP_LOGE(TAG, "USB client setup failed");
    state_.store(ST_FAILED);
    vTaskDelete(nullptr);
    return;
  }
  for (auto &x : in_xfer_) {
    if (usb_host_transfer_alloc(IN_BYTES, 0, &x) != ESP_OK) {
      ESP_LOGE(TAG, "USB transfer alloc failed");
      state_.store(ST_FAILED);
      vTaskDelete(nullptr);
      return;
    }
  }
  ESP_LOGI(TAG, "USB host ready, waiting for the CANable");

  for (;;) {
    usb_host_client_handle_events(client_, pdMS_TO_TICKS(20));
    if ((dev_gone_ || close_pending_) && dev_ != nullptr) {
      close_device_();
    }
    if (dev_ == nullptr && new_addr_ >= 0) {
      uint8_t addr = new_addr_;
      new_addr_ = -1;
      dev_gone_ = false;
      if (!open_device_(addr))
        close_device_();
    }
    if (state_.load() == ST_RUNNING && test_frames_)
      test_send_();
  }
}

// ---- control transfers (client task only) -----------------------------------
void GsUsbFd::ctrl_cb_(usb_transfer_t *xfer) {
  GsUsbFd *self = (GsUsbFd *) xfer->context;
  self->ctrl_done_ = true;
  self->ctrl_inflight_ = false;
}

// One control transfer on EP0, waited for (handles client events meanwhile).
bool GsUsbFd::ctrl_raw_(uint8_t type, uint8_t request, uint16_t value, uint16_t index, void *data,
                        uint16_t len, uint32_t timeout_ms) {
  bool in = type & 0x80;
  if (ctrl_inflight_)
    return false;  // the previous one never came back (device leaving)
  usb_setup_packet_t *s = (usb_setup_packet_t *) ctrl_xfer_->data_buffer;
  s->bmRequestType = type;
  s->bRequest = request;
  s->wValue = value;
  s->wIndex = index;
  s->wLength = len;
  if (!in && len)
    std::memcpy(ctrl_xfer_->data_buffer + sizeof(usb_setup_packet_t), data, len);
  ctrl_xfer_->num_bytes = sizeof(usb_setup_packet_t) + len;
  ctrl_xfer_->device_handle = dev_;
  ctrl_xfer_->bEndpointAddress = 0;
  ctrl_xfer_->callback = ctrl_cb_;
  ctrl_xfer_->context = this;
  ctrl_xfer_->timeout_ms = timeout_ms;
  ctrl_done_ = false;
  if (usb_host_transfer_submit_control(client_, ctrl_xfer_) != ESP_OK)
    return false;
  ctrl_inflight_ = true;
  int64_t t0 = esp_timer_get_time();
  while (!ctrl_done_ && esp_timer_get_time() - t0 < (int64_t) timeout_ms * 1000 + 500000)
    usb_host_client_handle_events(client_, pdMS_TO_TICKS(10));
  if (!ctrl_done_ || ctrl_xfer_->status != USB_TRANSFER_STATUS_COMPLETED)
    return false;
  if (in && len) {
    int got = ctrl_xfer_->actual_num_bytes - (int) sizeof(usb_setup_packet_t);
    std::memset(data, 0, len);
    std::memcpy(data, ctrl_xfer_->data_buffer + sizeof(usb_setup_packet_t),
                got > 0 ? std::min<int>(got, len) : 0);
  }
  return true;
}

// Vendor request, then GetLastError for its feedback (Elmue's firmware can't
// stall in the status stage, so that's how it reports a rejected request).
bool GsUsbFd::ctrl_(uint8_t request, bool in, void *data, uint16_t len) {
  const uint8_t vendor_if = 0x40 | 0x01;  // vendor, recipient interface; wValue = channel 0
  if (!ctrl_raw_((in ? 0x80 : 0x00) | vendor_if, request, 0, iface_, data, len)) {
    ESP_LOGW(TAG, "request %u failed (status %d)", request, ctrl_done_ ? (int) ctrl_xfer_->status : -1);
    return false;
  }
  if (request == REQ_GET_LAST_ERROR)
    return true;
  uint8_t fb = 0;
  if (!ctrl_raw_(0x80 | vendor_if, REQ_GET_LAST_ERROR, 0, iface_, &fb, 1)) {
    ESP_LOGW(TAG, "no feedback for request %u", request);
    return false;
  }
  if (fb != FBK_SUCCESS) {
    ESP_LOGW(TAG, "request %u rejected by the adapter (feedback %u)", request, fb);
    return false;
  }
  return true;
}

// The STM32's ROM DFU bootloader (DfuSe): point at the application and "leave",
// which starts the firmware already in flash (what `dfu-util -s 0x08000000:leave`
// does). Nothing is written. The CANable then re-enumerates as gs_usb.
bool GsUsbFd::leave_dfu_() {
  enum : uint8_t { DFU_DNLOAD = 1, DFU_GETSTATUS = 3, DFU_CLRSTATUS = 4, DFU_ABORT = 6 };
  const uint8_t out = 0x21, in = 0xA1;  // class, interface 0
  uint8_t st[6] = {};
  auto status = [&]() {
    if (!ctrl_raw_(in, DFU_GETSTATUS, 0, 0, st, sizeof(st)))
      return false;
    uint32_t poll = st[1] | (st[2] << 8) | (st[3] << 16);
    if (poll)
      vTaskDelay(pdMS_TO_TICKS(std::min<uint32_t>(poll, 500)));
    return true;
  };
  if (!status())
    return false;
  if (st[4] == 10)  // dfuERROR
    ctrl_raw_(out, DFU_CLRSTATUS, 0, 0, nullptr, 0);
  ctrl_raw_(out, DFU_ABORT, 0, 0, nullptr, 0);  // back to dfuIDLE
  uint8_t set_addr[5] = {0x21, 0x00, 0x00, 0x00, 0x08};  // Set Address Pointer 0x08000000
  if (!ctrl_raw_(out, DFU_DNLOAD, 0, 0, set_addr, sizeof(set_addr)) || !status() || !status())
    return false;
  ctrl_raw_(out, DFU_DNLOAD, 0, 0, nullptr, 0);  // zero-length download: leave
  status();  // the bootloader starts the application here, so this may fail
  return true;
}

// ---- open / configure / close -----------------------------------------------
bool GsUsbFd::open_device_(uint8_t addr) {
  state_.store(ST_OPENING);
  if (usb_host_device_open(client_, addr, &dev_) != ESP_OK) {
    dev_ = nullptr;
    return false;
  }
  const usb_device_desc_t *dd = nullptr;
  usb_host_get_device_descriptor(dev_, &dd);
  if (dd != nullptr && dd->idVendor == STM32_DFU_VID && dd->idProduct == STM32_DFU_PID) {
    // A CANable that came up in its bootloader (BOOT0 high at power-up)
    ESP_LOGW(TAG, "STM32 in DFU bootloader mode: starting its firmware");
    leave_dfu_();
    return false;  // it re-enumerates (DEV_GONE, then NEW_DEV as gs_usb)
  }
  if (dd == nullptr || dd->idVendor != CANABLE_VID || dd->idProduct != CANABLE_PID) {
    ESP_LOGW(TAG, "USB device %04X:%04X is not a gs_usb CANable (%04X:%04X), ignored",
             dd ? dd->idVendor : 0, dd ? dd->idProduct : 0, CANABLE_VID, CANABLE_PID);
    return false;
  }
  const usb_config_desc_t *cfg = nullptr;
  usb_host_get_active_config_descriptor(dev_, &cfg);
  int off = 0;
  const usb_intf_desc_t *intf = cfg ? usb_parse_interface_descriptor(cfg, 0, 0, &off) : nullptr;
  if (intf == nullptr || intf->bNumEndpoints != 2) {
    ESP_LOGW(TAG, "CANable: no gs_usb interface 0 with 2 endpoints");
    return false;
  }
  iface_ = intf->bInterfaceNumber;
  ep_in_ = ep_out_ = 0;
  for (int i = 0; i < 2; i++) {
    int eoff = off;
    const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, i, cfg->wTotalLength, &eoff);
    if (ep == nullptr || USB_EP_DESC_GET_XFERTYPE(ep) != USB_TRANSFER_TYPE_BULK)
      continue;
    if (USB_EP_DESC_GET_EP_DIR(ep))
      ep_in_ = ep->bEndpointAddress;
    else
      ep_out_ = ep->bEndpointAddress;
  }
  if (ep_in_ == 0 || ep_out_ == 0) {
    ESP_LOGW(TAG, "CANable: bulk endpoints not found");
    return false;
  }
  if (usb_host_interface_claim(client_, dev_, iface_, 0) != ESP_OK) {
    ESP_LOGW(TAG, "CANable: interface claim failed");
    return false;
  }
  iface_claimed_ = true;
  if (!configure_())
    return false;

  // stamps restart from the adapter's counter; re-learn the offset
  have_offset_ = false;
  win_min_ = INT64_MAX;
  win_t_ = 0;
  dev_wrap_ = 0;
  last_dev_ts_ = 0;
  if (was_running_)
    gap_pending_ = true;  // reconnected while capturing: frames were missed
  for (auto *x : in_xfer_)
    submit_in_(x);
  state_.store(ST_RUNNING);
  was_running_ = true;
  test_sent_ = test_rx_ = test_seq_err_ = test_next_rx_ = 0;
  test_t0_ = test_t1_ = 0;
  test_reported_ = false;
  return true;
}

bool GsUsbFd::configure_() {
  DeviceMode reset = {0, DEV_PROTOCOL_ELMUE};
  if (!ctrl_(REQ_SET_DEVICE_MODE, false, &reset, sizeof(reset)))
    return false;
  uint8_t cap_classic[40];
  if (!ctrl_(REQ_GET_CAPABILITIES, true, cap_classic, sizeof(cap_classic)))
    return false;
  uint32_t feat = rd32(cap_classic);
  const uint32_t need = DEV_LISTEN_ONLY | DEV_TIMESTAMP | DEV_PROTOCOL_ELMUE | DEV_SEND_USB_BLOBS |
                        (data_ ? DEV_CAN_FD | DEV_BIT_TIMING_FD : 0);
  if ((feat & need) != need) {
    ESP_LOGE(TAG, "CANable firmware lacks features 0x%05X (has 0x%05X): needs Elmue's CANable 2.5",
             (unsigned) (need & ~feat), (unsigned) feat);
    state_.store(ST_FAILED);
    return false;
  }
  CapabilityFD cap = {};
  if (!ctrl_(REQ_GET_CAPABILITIES_FD, true, &cap, sizeof(cap)))
    return false;
  DeviceVersion ver = {};
  BoardInfo bi = {};
  ctrl_(REQ_GET_DEVICE_VERSION, true, &ver, sizeof(ver));
  if (ctrl_(REQ_GET_BOARD_INFO, true, &bi, sizeof(bi))) {
    bi.board[24] = 0;
    bi.mcu[24] = 0;
    snprintf(adapter_, sizeof(adapter_), "%s %s fw %X", bi.board, bi.mcu, (unsigned) ver.sw_bcd);
  }

  BitTiming nt = {}, dt = {};
  if (!calc_timing(cap.fclk_can, nominal_, sp_nom_, cap.nom, nt)) {
    ESP_LOGE(TAG, "no bit timing for %u bit/s at %u Hz", (unsigned) nominal_, (unsigned) cap.fclk_can);
    state_.store(ST_FAILED);
    return false;
  }
  if (!ctrl_(REQ_SET_BIT_TIMING, false, &nt, sizeof(nt)))
    return false;
  if (data_) {
    if (!calc_timing(cap.fclk_can, data_, sp_data_, cap.data, dt)) {
      ESP_LOGE(TAG, "no data bit timing for %u bit/s", (unsigned) data_);
      state_.store(ST_FAILED);
      return false;
    }
    if (!ctrl_(REQ_SET_BIT_TIMING_FD, false, &dt, sizeof(dt)))
      return false;
  }
  uint8_t interval = 10;  // bus load report every 1 s
  ctrl_(REQ_SET_BUSLOAD_REPORT, false, &interval, 1);

  DeviceMode start = {1, DEV_LISTEN_ONLY | DEV_TIMESTAMP | DEV_PROTOCOL_ELMUE | DEV_SEND_USB_BLOBS};
  if (test_frames_)
    start.flags |= DEV_LOOPBACK;  // with LISTEN_ONLY: internal loopback, nothing on the wires
  if (!ctrl_(REQ_SET_DEVICE_MODE, false, &start, sizeof(start)))
    return false;
  ESP_LOGI(TAG, "CANable %s: %s, %u/%u bit/s (clock %u Hz, nominal brp %u seg %u/%u, data brp %u seg %u/%u)",
           adapter_, test_frames_ ? "INTERNAL LOOPBACK TEST" : "listen-only", (unsigned) nominal_,
           (unsigned) data_, (unsigned) cap.fclk_can, (unsigned) nt.brp, (unsigned) nt.seg1,
           (unsigned) nt.seg2, (unsigned) dt.brp, (unsigned) dt.seg1, (unsigned) dt.seg2);
  return true;
}

void GsUsbFd::close_device_() {
  if (dev_ == nullptr)
    return;
  bool was = state_.load() == ST_RUNNING;
  if (state_.load() != ST_FAILED)
    state_.store(ST_GONE);
  // in-flight transfers come back (NO_DEVICE or cancelled) before the interface
  // and the device can go
  if (iface_claimed_ && (inflight_ > 0 || out_busy_)) {
    for (uint8_t ep : {ep_in_, ep_out_}) {
      if (ep) {
        usb_host_endpoint_halt(dev_, ep);
        usb_host_endpoint_flush(dev_, ep);
      }
    }
  }
  int64_t t0 = esp_timer_get_time();
  while ((inflight_ > 0 || out_busy_ || ctrl_inflight_) && esp_timer_get_time() - t0 < 3000000)
    usb_host_client_handle_events(client_, pdMS_TO_TICKS(10));
  if (inflight_ > 0 || out_busy_ || ctrl_inflight_) {
    // closing now would assert in the USB host library; try again later
    ESP_LOGW(TAG, "USB transfers still in flight, closing later");
    close_pending_ = true;
    return;
  }
  close_pending_ = false;
  if (iface_claimed_) {
    usb_host_interface_release(client_, dev_, iface_);
    iface_claimed_ = false;
  }
  usb_host_device_close(client_, dev_);
  dev_ = nullptr;
  dev_gone_ = false;
  if (was)
    ESP_LOGW(TAG, "CANable disconnected");
}

// ---- bulk IN ------------------------------------------------------------------
void GsUsbFd::submit_in_(usb_transfer_t *x) {
  x->device_handle = dev_;
  x->bEndpointAddress = ep_in_;
  x->num_bytes = IN_BYTES;
  x->callback = in_cb_;
  x->context = this;
  x->timeout_ms = 0;
  if (usb_host_transfer_submit(x) == ESP_OK)
    inflight_++;
}

void GsUsbFd::in_cb_(usb_transfer_t *x) {
  GsUsbFd *self = (GsUsbFd *) x->context;
  self->inflight_--;
  if (x->status == USB_TRANSFER_STATUS_COMPLETED) {
    self->parse_(x->data_buffer, x->actual_num_bytes, esp_timer_get_time());
    if (self->state_.load() == ST_RUNNING && !self->dev_gone_)
      self->submit_in_(x);
  } else if (x->status != USB_TRANSFER_STATUS_NO_DEVICE && x->status != USB_TRANSFER_STATUS_CANCELED &&
             self->state_.load() == ST_RUNNING && !self->dev_gone_) {
    ESP_LOGW(TAG, "bulk IN status %d", (int) x->status);
    self->submit_in_(x);
  }
}

// A transfer holds one message, or a blob: {count, MSG_RX_BLOB} then count messages.
void GsUsbFd::parse_(const uint8_t *p, int n, int64_t now) {
  if (n >= 2 && p[1] == MSG_RX_BLOB) {
    int count = p[0], off = 2;
    for (int i = 0; i < count && off + 2 <= n; i++) {
      uint8_t size = p[off];
      if (size < 2 || off + size > n)
        break;
      handle_msg_(p + off, size, now);
      off += size;
    }
    return;
  }
  int off = 0;
  while (off + 2 <= n) {
    uint8_t size = p[off];
    if (size < 2 || off + size > n)
      break;
    handle_msg_(p + off, size, now);
    off += size;
  }
}

void GsUsbFd::handle_msg_(const uint8_t *m, uint8_t size, int64_t now) {
  switch (m[1]) {
    case MSG_RX_FRAME:
      rx_frame_(m, size, now);
      break;
    case MSG_ERROR: {  // {size, type, err_id u32, data[8], ts u32}
      if (size < 14)
        break;
      uint32_t err_id = rd32(m + 2);
      const uint8_t *d = m + 6;
      if (d[5] & (APP_CAN_RX_FAIL | APP_USB_IN_OVERFLOW)) {
        // the adapter dropped received frames since its last report
        loss_events_.fetch_add(1);
        gap_pending_ = true;
        ESP_LOGW(TAG, "CANable lost frames (flags 0x%02X)", d[5]);
      }
      uint8_t st = 0;
      if (err_id & ERID_BUS_OFF)
        st = 3;
      else if (d[1] & (ER1_RX_PASSIVE | ER1_TX_PASSIVE))
        st = 2;
      else if (d[1] & (ER1_RX_WARN | ER1_TX_WARN))
        st = 1;
      bus_status_.store(st);
      break;
    }
    case MSG_BUSLOAD:
      if (size >= 3) {
        busload_.store(m[2]);
        busload_t_.store(now);
      }
      break;
    case MSG_STRING:
      if (now - last_string_log_ > 1000000) {  // its debug messages, at most one a second
        last_string_log_ = now;
        char s[80];
        int len = std::min<int>(size - 2, sizeof(s) - 1);
        std::memcpy(s, m + 2, len);
        s[len] = 0;
        ESP_LOGI(TAG, "CANable: %s", s);
      }
      break;
    default:  // MSG_TX_ECHO: not requested (marker 0)
      break;
  }
}

// {size, type, flags, can_id u32, timestamp u32, data...}
void GsUsbFd::rx_frame_(const uint8_t *m, uint8_t size, int64_t now) {
  if (size < 11)
    return;
  uint8_t fl = m[2];
  uint32_t cid = rd32(m + 3);
  if (cid & ID_ERROR)
    return;
  FdFrame f;
  f.ts_us = map_ts_(rd32(m + 7), now);
  f.id = (cid & 0x1FFFFFFFu) | ((cid & ID_EXT) ? (1u << 31) : 0) | ((cid & ID_RTR) ? (1u << 30) : 0);
  f.flags = ((fl & FRM_FDF) ? FD_F_FDF : 0) | ((fl & FRM_BRS) ? FD_F_BRS : 0) | ((fl & FRM_ESI) ? FD_F_ESI : 0);
  uint8_t len = size - 11;
  if (len > 64)
    len = 64;
  f.len = len;
  std::memcpy(f.data, m + 11, len);
  if (len < 64)
    std::memset(f.data + len, 0, 64 - len);
  if (cid & ID_RTR) {  // RTR: the DLC is sent in the first data byte
    f.len = len ? m[11] : 0;
    std::memset(f.data, 0, sizeof(f.data));
  }
  push_(f);

  if (test_frames_ && (f.id & 0x1FFFFFFFu) == ((f.id & (1u << 31)) ? 0x1FD1FD0u : TEST_ID)) {
    uint32_t seq = f.len >= 4 ? rd32(f.data) : test_next_rx_;  // short frames carry no number
    if (seq != test_next_rx_)
      test_seq_err_++;
    test_next_rx_ = seq + 1;
    test_rx_++;
    test_t1_ = now;
  }
}

// The adapter's 32-bit 1 us counter (wraps after ~71 min), unwrapped and moved
// onto esp_timer time. The offset is the smallest host-minus-device difference
// seen (the least-delayed transfer); after the first 5 s it may only move
// 200 us per second, so latency jitter can't shake it but crystal drift is followed.
uint64_t GsUsbFd::map_ts_(uint32_t dev_ts, int64_t now) {
  if (dev_ts < last_dev_ts_ && last_dev_ts_ - dev_ts > 0x80000000u)
    dev_wrap_ += 1ull << 32;
  last_dev_ts_ = dev_ts;
  int64_t dev = (int64_t) (dev_wrap_ + dev_ts);
  int64_t cand = now - dev;
  if (!have_offset_) {
    have_offset_ = true;
    offset_ = cand;
    win_t_ = first_t_ = now;
  }
  if (cand < win_min_)
    win_min_ = cand;
  if (now - win_t_ >= 1000000) {
    int64_t step = win_min_ - offset_;
    if (now - first_t_ > 5000000)
      step = std::max<int64_t>(-200, std::min<int64_t>(200, step));
    offset_ += step;
    win_min_ = INT64_MAX;
    win_t_ = now;
  }
  uint64_t ts = (uint64_t) (dev + offset_);
  if (ts < last_host_ts_)
    ts = last_host_ts_;  // keep the channel's own order
  last_host_ts_ = ts;
  return ts;
}

// ---- loopback test sender ---------------------------------------------------
void GsUsbFd::out_cb_(usb_transfer_t *x) { ((GsUsbFd *) x->context)->out_busy_ = false; }

// Numbered frames, cycling through every FD length, standard and extended ids,
// with every 17th a classic 8-byte frame. At most 32 in flight (the adapter
// rejects a whole blob that doesn't fit its 64-frame queue).
void GsUsbFd::test_send_() {
  if (test_sent_ >= test_frames_) {
    if (!test_reported_ && (test_rx_ >= test_sent_ || esp_timer_get_time() - test_t1_ > 2000000)) {
      test_reported_ = true;
      float dt = (test_t1_ - test_t0_) / 1e6f;
      ESP_LOGI(TAG, "FD loopback test: sent %u, received %u, sequence errors %u, %.0f frames/s",
               (unsigned) test_sent_, (unsigned) test_rx_, (unsigned) test_seq_err_,
               dt > 0 ? test_rx_ / dt : 0.0f);
    }
    return;
  }
  if (out_busy_ || test_sent_ - test_rx_ > 16)
    return;
  if (test_t0_ == 0)
    test_t0_ = esp_timer_get_time();
  uint8_t *b = out_xfer_->data_buffer;
  int off = 2, count = 0;
  while (count < 16 && test_sent_ < test_frames_ && test_sent_ - test_rx_ + count < 32) {
    uint32_t seq = test_sent_ + count;
    bool classic = (seq % 17) == 16;  // 17 so the FD lengths include 64
    bool ext = (seq % 5) == 4;
    uint8_t len = classic ? 8 : FD_LENS[seq % 17];
    if (len < 4)
      len = 4;  // every test frame carries its number
    uint8_t *m = b + off;
    m[0] = 8 + len;
    m[1] = MSG_TX_FRAME;
    m[2] = classic ? 0 : (FRM_FDF | FRM_BRS);
    wr32(m + 3, ext ? (0x1FD1FD0u | ID_EXT) : TEST_ID);
    m[7] = 0;  // marker 0: no echo
    wr32(m + 8, seq);
    for (uint8_t i = 4; i < len; i++)
      m[8 + i] = (uint8_t) (seq * 7 + i);
    off += 8 + len;
    count++;
  }
  b[0] = count;
  b[1] = MSG_TX_BLOB;
  out_xfer_->device_handle = dev_;
  out_xfer_->bEndpointAddress = ep_out_;
  out_xfer_->num_bytes = off;
  out_xfer_->flags = USB_TRANSFER_FLAG_ZERO_PACK;
  out_xfer_->callback = out_cb_;
  out_xfer_->context = this;
  out_xfer_->timeout_ms = 0;
  out_busy_ = true;
  if (usb_host_transfer_submit(out_xfer_) == ESP_OK)
    test_sent_ += count;
  else
    out_busy_ = false;
}

// ---- status -------------------------------------------------------------------
float GsUsbFd::bus_load_pct() const {
  int64_t t = busload_t_.load();
  if (t == 0 || esp_timer_get_time() - t > 3000000)
    return 0;  // the adapter only reports while there's traffic
  return busload_.load();
}

const char *GsUsbFd::state() const {
  switch (state_.load()) {
    case ST_NONE: return "no adapter";
    case ST_OPENING: return "starting";
    case ST_FAILED: return "failed";
    case ST_GONE: return "disconnected";
    default: break;
  }
  static const char *const st[] = {"active", "warning", "passive", "bus-off"};
  return st[bus_status_.load() & 3];
}

std::string GsUsbFd::adapter() const { return adapter_[0] ? adapter_ : "-"; }

}  // namespace can_logger
}  // namespace esphome

#endif  // USE_CAN_LOGGER_GSUSB
