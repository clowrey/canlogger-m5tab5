#!/usr/bin/env python3
"""Log every CAN frame from a CANable to a candump (-l) log, for cross-checking
the Tab5 capture (see canbin2candump.py, which writes the same line format).

    python canable_log.py                      # auto-detect, 500 kbit/s
    python canable_log.py --backend slcan --port COM19 --allow-ack
    python canable_log.py -o hvc_flash.log --bitrate 500000

Two firmwares:
  gs_usb (candleLight)  opened listen-only (bus monitoring): it never ACKs or
                        sends on the bus. Hardware timestamps when the firmware
                        fills them (budgetcan_fw doesn't: PC time then).
  slcan (factory)       the canable2 slcan firmware ignores its silent command
                        ('M1': can_enable() resets the mode to normal), so the
                        CANable ACKs every frame like an ECU would. Needs
                        --allow-ack; timestamps are taken on the PC.

Stop with Ctrl+C. The file is flushed every second. "CANable overflows" counts
the times the CANable reported dropping frames itself (its queue to the PC ran
full); the log is only a complete reference while that stays 0.
Needs: pip install gs_usb libusb-package pyserial
"""
import argparse
import queue
import sys
import threading
import time

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_ERR_CRTL = 0x00000004            # error frame class: controller problem, details in data[1]
CAN_ERR_CRTL_RX_OVERFLOW = 0x01

SLCAN_BITRATES = {10000: 0, 20000: 1, 50000: 2, 100000: 3, 125000: 4,
                  250000: 5, 500000: 6, 750000: 7, 1000000: 8}
CANABLE_SLCAN_VID_PID = (0x16D0, 0x117E)


class FrameClock:
    """Timestamps for gs_usb frames. The device's 32-bit microsecond clock wraps
    every ~71 min: unwrap it and anchor it to the PC clock at the first frame.
    Firmware that offers hardware timestamps but sends 0 (budgetcan_fw does)
    falls back to PC time."""

    def __init__(self, hw):
        self.hw = hw
        self.anchor = self.last = None
        self.total = 0

    def stamp(self, ts_us):
        if self.hw and self.last is None and ts_us == 0:
            self.hw = False
            print("device timestamps are all 0: using PC timestamps", file=sys.stderr)
        if not self.hw:
            return time.time()
        if self.last is None:
            self.anchor, self.last = time.time(), ts_us
        self.total += (ts_us - self.last) & 0xFFFFFFFF
        self.last = ts_us
        return self.anchor + self.total / 1e6


def candump_line(ts, iface, can_id, ext, rtr, data):
    idstr = f"{can_id:08X}" if ext else f"{can_id:03X}"
    body = f"{idstr}#R" if rtr else f"{idstr}#{data.hex().upper()}"
    return f"({ts:.6f}) {iface} {body}\n"


# ---- gs_usb (candleLight) ------------------------------------------------
def find_gs_usb():
    try:
        import libusb_package
        import usb.backend.libusb1 as libusb1
        from gs_usb.gs_usb import GsUsb
    except ImportError:
        return None
    # gs_usb asks pyusb for the default libusb backend, which Windows lacks;
    # point it at the DLL shipped by libusb-package.
    backend = libusb_package.get_libusb1_backend()
    libusb1.get_backend = lambda *a, **k: backend
    devs = GsUsb.scan()
    if not devs:
        return None
    try:
        devs[0].device_capability  # first control transfer opens the device
    except Exception as e:  # usb.core.USBError
        if getattr(e, "errno", None) == 13 or "Access denied" in str(e):
            sys.exit("CANable is busy: another program (another canable_* script?) has it open")
        raise
    return devs[0]


def set_bitrate(dev, bitrate, sample_point=0.875):
    """gs_usb's own table only knows 48 and 80 MHz CAN clocks; otherwise derive the
    timing from the device's clock and limits (16-25 quanta, sample point ~87.5 %)."""
    if dev.set_bitrate(bitrate):
        return True
    cap = dev.device_capability
    best = None
    for tq in range(8, 26):
        if cap.fclk_can % (bitrate * tq):
            continue
        brp = cap.fclk_can // (bitrate * tq)
        tseg2 = max(1, round(tq * (1 - sample_point)))
        tseg1 = tq - 1 - tseg2                    # prop_seg + phase_seg1
        if not (cap.brp_min <= brp <= cap.brp_max and cap.tseg1_min <= tseg1 <= cap.tseg1_max
                and cap.tseg2_min <= tseg2 <= cap.tseg2_max):
            continue
        key = (abs((tq - tseg2) / tq - sample_point), -tq)   # closest sample point, then more quanta
        if best is None or key < best[0]:
            best = (key, tseg1, tseg2, brp)
    if best is None:
        return False
    _, tseg1, tseg2, brp = best
    dev.set_timing(1, tseg1 - 1, tseg2, min(tseg2, cap.sjw_max), brp)
    return True


def drain_gs_usb(dev):
    """Drop frames still queued from an earlier run (they carry stale timestamps)."""
    import usb.core

    dropped = 0
    while True:
        try:
            dev.gs_usb.read(0x81, 64, 20)
        except usb.core.USBError:     # timeout: nothing left
            return dropped
        dropped += 1


class UsbReader:
    """Keeps n bulk IN transfers queued on the CANable (libusb async, on the
    handle pyusb already has open) and puts each received packet on q.

    gs_usb's read() submits one transfer at a time, and on Windows each round
    trip takes ~0.3 ms (a few ms at worst). The CANable holds only a few frames
    meanwhile, so on a busy bus (~2,300 frames/s, bursts 95 us apart) it
    overflowed and dropped bursts of 30-40 frames. With transfers waiting, each
    frame leaves on the next USB poll and the PC can fall n frames behind
    (n=64, ~28 ms of that bus, still lost one burst a minute to Windows
    scheduling; 256 gives ~110 ms)."""

    def __init__(self, dev, size, q, n=256):
        import ctypes
        import usb.backend.libusb1 as lu

        self.ct, self.lu, self.q = ctypes, lu, q
        res = dev.gs_usb._ctx
        res.managed_claim_interface(dev.gs_usb, 0)   # start()'s USB reset closed the handle
        self.lib, self.usb_ctx = res.backend.lib, res.backend.ctx
        self.lib.libusb_cancel_transfer.argtypes = [lu._libusb_transfer_p]
        self.lib.libusb_handle_events_timeout.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.stopping = False
        self.pending = 0
        self.cb = lu._libusb_transfer_cb_fn_p(self._done)   # must outlive the transfers
        self.bufs, self.xfers = [], []
        for _ in range(n):
            buf = ctypes.create_string_buffer(size)
            t = self.lib.libusb_alloc_transfer(0)
            c = t.contents
            c.dev_handle = res.handle.handle
            c.endpoint = 0x81
            c.type = 2                      # LIBUSB_TRANSFER_TYPE_BULK
            c.timeout = 0
            c.length = size
            c.callback = self.cb
            c.buffer = ctypes.cast(buf, ctypes.c_void_p)
            self.bufs.append(buf)
            self.xfers.append(t)
        for t in self.xfers:
            self._submit(t)
        self.thread = threading.Thread(target=self._events, daemon=True)
        self.thread.start()

    def _submit(self, t):
        rc = self.lib.libusb_submit_transfer(t)
        if rc:
            self.q.put(IOError(f"libusb_submit_transfer failed ({rc})"))
        else:
            self.pending += 1

    def _done(self, t):
        # runs in _events' thread, inside libusb_handle_events_timeout
        c = t.contents
        self.pending -= 1
        if c.status == self.lu.LIBUSB_TRANSFER_COMPLETED:
            self.q.put(self.ct.string_at(c.buffer, c.actual_length))
        elif c.status != self.lu.LIBUSB_TRANSFER_CANCELLED:
            self.q.put(IOError(f"USB transfer failed (libusb transfer status {c.status})"))
            self.stopping = True
            return
        if not self.stopping:
            self._submit(t)

    def _events(self):
        class Timeval(self.ct.Structure):
            _fields_ = [("tv_sec", self.ct.c_long), ("tv_usec", self.ct.c_long)]

        tv = Timeval(0, 100000)
        while not self.stopping:
            self.lib.libusb_handle_events_timeout(self.usb_ctx, self.ct.byref(tv))
        # cancel from this thread so no callback can resubmit behind our back
        for t in self.xfers:
            self.lib.libusb_cancel_transfer(t)
        deadline = time.time() + 2
        while self.pending and time.time() < deadline:
            self.lib.libusb_handle_events_timeout(self.usb_ctx, self.ct.byref(tv))

    def stop(self):
        self.stopping = True
        self.thread.join(3)
        if not self.pending:
            for t in self.xfers:
                self.lib.libusb_free_transfer(t)


def run_gs_usb(dev, args, out, stats):
    from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP, GS_CAN_MODE_LISTEN_ONLY
    from gs_usb.gs_usb_frame import GsUsbFrame

    feature = dev.device_capability.feature
    if not feature & GS_CAN_MODE_LISTEN_ONLY:
        sys.exit("this gs_usb firmware does not offer listen-only mode; not opening it")
    try:
        dev.stop()
    except Exception:
        pass
    stale = drain_gs_usb(dev)
    if stale:
        print(f"dropped {stale} frame(s) left over from an earlier run", file=sys.stderr)
    if not set_bitrate(dev, args.bitrate):
        sys.exit(f"gs_usb: bitrate {args.bitrate} not supported")
    flags = GS_CAN_MODE_LISTEN_ONLY
    hw_ts = bool(feature & GS_CAN_MODE_HW_TIMESTAMP)
    if hw_ts:
        flags |= GS_CAN_MODE_HW_TIMESTAMP
    dev.start(flags)
    print(f"gs_usb {dev} serial {dev.serial_number}: listen-only, "
          f"{'hardware' if hw_ts else 'PC'} timestamps", file=sys.stderr)

    clock = FrameClock(hw_ts)
    frame = GsUsbFrame()
    q = queue.SimpleQueue()
    reader = UsbReader(dev, frame.__sizeof__(hw_ts), q)
    try:
        while True:
            stats.tick(out)
            try:
                raw = q.get(timeout=0.2)
            except queue.Empty:
                continue
            if isinstance(raw, Exception):
                sys.exit(f"gs_usb read failed: {raw}")
            GsUsbFrame.unpack_into(frame, raw, hw_ts)
            if frame.echo_id != 0xFFFFFFFF:     # echo of our own TX (never in listen-only)
                continue
            if frame.can_id & CAN_ERR_FLAG:
                stats.errors += 1
                if frame.can_id & CAN_ERR_CRTL and frame.data[1] & CAN_ERR_CRTL_RX_OVERFLOW:
                    stats.overflows += 1
                continue
            ts = clock.stamp(frame.timestamp_us)
            dlc = min(frame.can_dlc, 8)
            out.write(candump_line(ts, args.iface, frame.can_id & CAN_EFF_MASK,
                                   bool(frame.can_id & CAN_EFF_FLAG),
                                   bool(frame.can_id & CAN_RTR_FLAG),
                                   bytes(frame.data[:dlc])))
            stats.frames += 1
    finally:
        reader.stop()
        dev.stop()


# ---- slcan (factory canable2 firmware) -------------------------------------
def find_slcan_port():
    from serial.tools import list_ports
    for p in list_ports.comports():
        if (p.vid, p.pid) == CANABLE_SLCAN_VID_PID:
            return p.device
    return None


def parse_slcan(line):
    """'t1238DEADBEEF00000000' -> (id, ext, rtr, data), or None."""
    kind = line[:1]
    if kind in ("t", "r"):
        n, ext = 3, False
    elif kind in ("T", "R"):
        n, ext = 8, True
    else:
        return None                     # FD frames (d/D/b/B) and replies
    can_id = int(line[1:1 + n], 16)
    dlc = int(line[1 + n], 16)
    rtr = kind in ("r", "R")
    data = b"" if rtr else bytes.fromhex(line[2 + n:2 + n + 2 * dlc])
    return can_id, ext, rtr, data


def run_slcan(port, args, out, stats):
    import serial

    s = serial.Serial(port, 115200, timeout=0.2)
    s.write(b"C\r")
    s.reset_input_buffer()
    s.write(b"V\r")
    time.sleep(0.2)
    version = s.read(200).decode(errors="replace").strip()
    s.write(f"S{SLCAN_BITRATES[args.bitrate]}\r".encode())
    s.write(b"O\r")
    print(f"slcan {port} ({version}): normal mode (ACKs frames), PC timestamps",
          file=sys.stderr)
    buf = b""
    try:
        while True:
            stats.tick(out)
            chunk = s.read(s.in_waiting or 1)
            if not chunk:
                continue
            ts = time.time()
            buf += chunk
            *lines, buf = buf.split(b"\r")
            for raw in lines:
                line = raw.decode(errors="replace").strip("\x07\n")
                if not line:
                    continue
                try:
                    f = parse_slcan(line)
                except (ValueError, IndexError):
                    f = None
                if f is None:
                    stats.junk += 1
                    continue
                out.write(candump_line(ts, args.iface, *f))
                stats.frames += 1
    finally:
        s.write(b"C\r")
        s.close()


class Stats:
    def __init__(self, every, duration):
        self.frames = self.errors = self.overflows = self.junk = 0
        self.every = every
        self.duration = duration
        self.t0 = self.t_flush = self.t_print = time.time()
        self.last_frames = 0

    def tick(self, out):
        now = time.time()
        if self.duration and now - self.t0 >= self.duration:
            raise KeyboardInterrupt
        if now - self.t_flush >= 1.0:
            out.flush()
            self.t_flush = now
        if now - self.t_print >= self.every:
            fps = (self.frames - self.last_frames) / (now - self.t_print)
            print(f"{now - self.t0:7.0f} s  frames {self.frames}  ({fps:.0f}/s)  "
                  f"error frames {self.errors}  CANable overflows {self.overflows}  "
                  f"unparsed {self.junk}", file=sys.stderr)
            self.t_print, self.last_frames = now, self.frames


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-o", "--out", help="log file (default candump-<date>-<time>.log)")
    ap.add_argument("-b", "--bitrate", type=int, default=500000)
    ap.add_argument("-i", "--iface", default="can0", help="interface name written in the log")
    ap.add_argument("--backend", choices=("auto", "gs_usb", "slcan"), default="auto")
    ap.add_argument("--port", help="slcan COM port (default: find the CANable)")
    ap.add_argument("--allow-ack", action="store_true",
                    help="slcan only: accept that the CANable ACKs frames on the bus")
    ap.add_argument("--stats-every", type=float, default=5.0, help="seconds between status lines")
    ap.add_argument("--duration", type=float, default=0, help="stop after this many seconds (0 = run until Ctrl+C)")
    args = ap.parse_args()

    dev = port = None
    if args.backend in ("auto", "gs_usb"):
        dev = find_gs_usb()
    if dev is None and args.backend in ("auto", "slcan"):
        port = args.port or find_slcan_port()
    if dev is None and port is None:
        sys.exit("no CANable found (gs_usb or slcan)")
    if dev is None:
        if args.bitrate not in SLCAN_BITRATES:
            sys.exit(f"slcan: bitrate {args.bitrate} not supported")
        if not args.allow_ack:
            sys.exit("The factory slcan firmware cannot listen silently: it would ACK frames on\n"
                     "the bus. Flash candleLight (gs_usb) for listen-only, or pass --allow-ack.")

    name = args.out or time.strftime("candump-%Y%m%d-%H%M%S.log")
    stats = Stats(args.stats_every, args.duration)
    with open(name, "w", newline="\n") as out:
        print(f"logging to {name} at {args.bitrate} bit/s; Ctrl+C to stop", file=sys.stderr)
        try:
            if dev is not None:
                run_gs_usb(dev, args, out, stats)
            else:
                run_slcan(port, args, out, stats)
        except KeyboardInterrupt:
            pass
    print(f"stopped: {stats.frames} frames, {stats.errors} error frames, "
          f"{stats.overflows} CANable overflows -> {name}", file=sys.stderr)


if __name__ == "__main__":
    main()
