#!/usr/bin/env python3
"""Send numbered CAN FD test frames from a CANable with Elmue's Candlelight 2.5,
for checking the Tab5's FD channel on a desk bus where nothing else transmits.

    python canable_fd_testgen.py -n 20000 -p fd1              # every length, mixed
    python canable_fd_testgen.py -n 20000 --len 64 -p fd64    # 64-byte frames only
    python canable_fd_testgen.py -n 20000 --data-bitrate 5000000 -p fd5m

The CANable runs in external loopback: it drives the bus but does not need an
ACK (both Tab5 channels only listen), and it reads its own frames back.
Never use this on a car: it transmits.

Writes <prefix>_sent.log (what was queued) and <prefix>_rx.log (the CANable's
own copies) in candump -l format, CAN FD as ID##<flags>DATA, so the capture can
be checked with tools/compare_logs.py --iface can1 exact. The CANable's bit
timing must match fd_channel's: the sample points here are the same defaults.
"""
import argparse
import collections
import struct
import sys
import threading
import time

import libusb_package
import usb.core
import usb.util

VID, PID = 0x1D50, 0x606F
REQ_SET_BIT_TIMING, REQ_SET_DEVICE_MODE, REQ_GET_CAPABILITIES_FD = 1, 2, 11
REQ_SET_BIT_TIMING_FD, REQ_GET_LAST_ERROR = 10, 22
FBK_SUCCESS = 2
DEV_LOOPBACK, DEV_TIMESTAMP, DEV_CAN_FD, DEV_BIT_TIMING_FD = 0x2, 0x10, 0x100, 0x400
DEV_PROTOCOL_ELMUE, DEV_SEND_USB_BLOBS = 0x4000, 0x8000
MSG_TX_FRAME, MSG_TX_ECHO, MSG_RX_FRAME, MSG_ERROR, MSG_STRING, MSG_TX_BLOB, MSG_RX_BLOB = (
    0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x10, 0x11)
FRM_FDF, FRM_BRS, FRM_ESI = 0x02, 0x04, 0x08
ID_RTR, ID_EXT = 0x40000000, 0x80000000
APP_NAMES = {0x01: "CAN RX failed", 0x02: "CAN TX failed", 0x04: "USB OUT overflow", 0x08: "USB IN overflow"}
FD_LENS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)
MAX_BLOB = 2048
BLOB_FRAMES = 16


def calc_timing(fclk, bitrate, sp, lim, only_brp=None):
    """(seg1, seg2, sjw, brp) like gs_usb_fd.cpp's calc_timing. lim: the 8 limits."""
    seg1_min, seg1_max, seg2_min, seg2_max, sjw_max, brp_min, brp_max, _ = lim
    best = None
    for brp in range(max(brp_min, 1), min(brp_max, 1024) + 1):
        if fclk % (brp * bitrate) or (only_brp and brp != only_brp):
            continue
        tq = fclk // (brp * bitrate)
        if tq < 1 + seg1_min + seg2_min or tq > 1 + seg1_max + seg2_max:
            continue
        seg2 = max(seg2_min, min(seg2_max, tq - round(tq * sp)))
        seg1 = tq - 1 - seg2
        if not seg1_min <= seg1 <= seg1_max:
            continue
        err = abs((1 + seg1) / tq - sp)
        if best is None or err < best[0] - 1e-6:
            best = (err, seg1, seg2, min(seg1, seg2, sjw_max or seg2), brp)
    return best[1:] if best else None


def body(can_id, ext, rtr, fd, brs, data):
    """The text after the interface name, same as canbin.candump_body."""
    idstr = f"{can_id:08X}" if ext else f"{can_id:03X}"
    if rtr:
        return f"{idstr}#R"
    if fd:
        return f"{idstr}##{1 if brs else 0:X}{data.hex().upper()}"
    return f"{idstr}#{data.hex().upper()}"


def make_frame(i, args):
    """Frame i: (id, ext, rtr, fd, brs, data). The first bytes carry i, so a frame
    shorter than 4 bytes only holds part of it."""
    classic = args.classic or (not args.fd_only and args.len is None and i % 17 == 16)
    rtr = classic and i % (17 * 11) == 16
    ext = i % 5 == 4
    can_id = (0x18DA00F1 + (i % 7)) if ext else 0x100 + (i % 0x400)
    if classic:
        n = 0 if rtr else i % 9
        brs = False
    else:
        n = args.len if args.len is not None else FD_LENS[i % 16]
        brs = not args.no_brs
    tail = bytes((i * 7 + k) & 0xFF for k in range(4, max(n, 4)))
    return can_id, ext, rtr, not classic, brs, (i.to_bytes(4, "big") + tail)[:n]


class Sender:
    def __init__(self, args):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID, backend=libusb_package.get_libusb1_backend())
        if self.dev is None:
            sys.exit("no CANable with gs_usb firmware found")
        self.dev.reset()  # a run that ended badly can leave the bulk IN endpoint stuck
        self.dev.set_configuration()
        intf = self.dev.get_active_configuration()[(0, 0)]
        usb.util.claim_interface(self.dev, 0)
        for ep in intf:
            if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN:
                self.ep_in = ep.bEndpointAddress
            else:
                self.ep_out = ep.bEndpointAddress
        self.mode(0, 0)
        cap = struct.unpack("<II8I8I", bytes(self.dev.ctrl_transfer(0xC1, REQ_GET_CAPABILITIES_FD, 0, 0, 72, 1000)))
        need = DEV_CAN_FD | DEV_BIT_TIMING_FD | DEV_PROTOCOL_ELMUE | DEV_LOOPBACK
        if cap[0] & need != need:
            sys.exit(f"this firmware can't do it (features 0x{cap[0]:X}): it needs Elmue's Candlelight 2.5")
        fclk = cap[1]
        dt = calc_timing(fclk, args.data_bitrate, args.data_sample_point, cap[10:18])
        nt = calc_timing(fclk, args.bitrate, args.sample_point, cap[2:10],
                         dt[3] if dt and not args.own_prescalers else None)
        if nt is None or dt is None:
            sys.exit(f"no bit timing for {args.bitrate}/{args.data_bitrate} at {fclk} Hz")
        self.set_timing(REQ_SET_BIT_TIMING, nt)
        self.set_timing(REQ_SET_BIT_TIMING_FD, dt)
        print(f"clock {fclk} Hz; nominal seg1/seg2/sjw/brp {nt}, data {dt}")
        self.iface = args.iface
        self.sent_f = open(f"{args.prefix}_sent.log", "w", newline="\n")
        self.rx_f = open(f"{args.prefix}_rx.log", "w", newline="\n")
        self.nsent = self.nrx = 0
        self.marker = 0
        self.markers = collections.deque()    # markers in flight, oldest first
        self.nechoed = 0
        self.errors = {}
        self.strings = []
        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.mode(1, DEV_LOOPBACK | DEV_TIMESTAMP | DEV_CAN_FD | DEV_BIT_TIMING_FD | DEV_PROTOCOL_ELMUE
                  | DEV_SEND_USB_BLOBS)
        self.reader.start()

    def checked(self, request, payload):
        self.dev.ctrl_transfer(0x41, request, 0, 0, payload, 1000)
        r = self.dev.ctrl_transfer(0xC1, REQ_GET_LAST_ERROR, 0, 0, 1, 1000)
        if r[0] != FBK_SUCCESS:
            sys.exit(f"the CANable refused request {request}: feedback {r[0]}")

    def mode(self, mode, flags):
        self.checked(REQ_SET_DEVICE_MODE, struct.pack("<II", mode, flags))

    def set_timing(self, request, t):
        seg1, seg2, sjw, brp = t
        self.checked(request, struct.pack("<5I", 0, seg1, seg2, sjw, brp))

    def read_loop(self):
        while self.running:
            try:
                buf = bytes(self.dev.read(self.ep_in, MAX_BLOB, 1000))
            except usb.core.USBTimeoutError:
                continue
            except usb.core.USBError:
                if self.running:
                    raise
                return
            if len(buf) >= 2 and buf[1] == MSG_RX_BLOB:
                p = 2
                for _ in range(buf[0]):
                    self.message(buf[p:p + buf[p]])
                    p += buf[p]
            elif buf:
                self.message(buf)

    def message(self, m):
        kind = m[1]
        if kind == MSG_RX_FRAME:
            flags, can_id = m[2], struct.unpack_from("<I", m, 3)[0]
            ts = struct.unpack_from("<I", m, 7)[0]
            ext = bool(can_id & ID_EXT)
            cid = can_id & (0x1FFFFFFF if ext else 0x7FF)
            line = body(cid, ext, bool(can_id & ID_RTR), bool(flags & FRM_FDF), bool(flags & FRM_BRS), m[11:])
            self.rx_f.write(f"({ts / 1e6:.6f}) {self.iface} {line}\n")
            self.nrx += 1
        elif kind == MSG_TX_ECHO:
            # an echo can get lost like a read-back copy: everything older has gone too
            if m[2] in self.markers:
                while self.markers:
                    self.nechoed += 1
                    if self.markers.popleft() == m[2]:
                        break
        elif kind == MSG_ERROR:
            app = m[11] if len(m) > 11 else 0
            for bit, name in APP_NAMES.items():
                if app & bit:
                    self.errors[name] = self.errors.get(name, 0) + 1
            if struct.unpack_from("<I", m, 2)[0] or any(m[6:11]):
                self.errors["bus error report"] = self.errors.get("bus error report", 0) + 1
        elif kind == MSG_STRING:
            self.strings.append(m[2:].split(b"\0")[0].decode(errors="replace"))

    def send_blob(self, frames):
        out = bytearray(2)
        for can_id, ext, rtr, fd, brs, data in frames:
            flags = (FRM_FDF if fd else 0) | (FRM_BRS if fd and brs else 0)
            wire_id = can_id | (ID_EXT if ext else 0) | (ID_RTR if rtr else 0)
            self.marker = self.marker % 255 + 1
            self.markers.append(self.marker)
            out += struct.pack("<BBBIB", 8 + len(data), MSG_TX_FRAME, flags, wire_id, self.marker) + data
        out[0], out[1] = len(frames), MSG_TX_BLOB
        self.dev.write(self.ep_out, bytes(out), 1000)
        if len(out) % 64 == 0:
            self.dev.write(self.ep_out, b"", 1000)  # the end of the transfer, or the next blob joins it and is lost
        for can_id, ext, rtr, fd, brs, data in frames:
            self.sent_f.write(f"({time.time():.6f}) {self.iface} {body(can_id, ext, rtr, fd, brs, data)}\n")
        self.nsent += len(frames)

    def close(self):
        deadline = time.time() + 1.5
        while time.time() < deadline and (self.nrx < self.nsent or self.markers):
            time.sleep(0.02)
        self.running = False
        self.reader.join(1)
        self.mode(0, 0)
        self.sent_f.close()
        self.rx_f.close()
        usb.util.release_interface(self.dev, 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-n", "--count", type=int, default=1000)
    ap.add_argument("-b", "--bitrate", type=int, default=500000)
    ap.add_argument("-d", "--data-bitrate", type=int, default=2000000)
    ap.add_argument("--sample-point", type=float, default=0.80)
    ap.add_argument("--data-sample-point", type=float, default=0.75)
    ap.add_argument("--own-prescalers", action="store_true",
                    help="let the nominal phase pick its own clock prescaler instead of the data phase's")
    ap.add_argument("--len", type=int, choices=FD_LENS, help="only FD frames of this length (default: mixed)")
    ap.add_argument("--fd-only", action="store_true", help="no classic frames among the FD ones")
    ap.add_argument("--no-brs", action="store_true", help="FD frames stay at the nominal rate")
    ap.add_argument("--classic", action="store_true", help="only classic frames")
    ap.add_argument("--gap-ms", type=float, default=0, help="pause between frames (0 = as fast as it goes)")
    ap.add_argument("--window", type=int, default=32, help="frames in flight")
    ap.add_argument("--start", type=int, default=0, help="first frame number")
    ap.add_argument("-p", "--prefix", default="fdgen")
    ap.add_argument("-i", "--iface", default="can1")
    args = ap.parse_args()

    tx = Sender(args)
    t0 = time.time()
    i = args.start
    end = args.start + args.count
    stalled = False
    try:
        while i < end:
            if args.gap_ms:
                n = 1
                time.sleep(args.gap_ms / 1000)
            else:
                n = min(BLOB_FRAMES, end - i)
            t = time.time()
            while len(tx.markers) + n > args.window and time.time() - t < 1.0:
                time.sleep(0.0002)
            if len(tx.markers) + n > args.window:
                stalled = True
                print(f"no TX echo for 1 s after {tx.nsent} frames: is the bus wired and terminated?")
                break
            tx.send_blob([make_frame(k, args) for k in range(i, i + n)])
            i += n
    finally:
        tx.close()
    dt = time.time() - t0
    print(f"sent {tx.nsent}, TX echoes {tx.nechoed}, read back {tx.nrx} in {dt:.1f} s "
          f"({tx.nsent / dt:.0f} frames/s) -> {args.prefix}_sent.log, {args.prefix}_rx.log")
    if tx.errors:
        print("CANable reports:", tx.errors)
    for s in tx.strings[:12]:
        print("CANable says:", s)
    sys.exit(1 if stalled or tx.nechoed != tx.nsent else 0)


if __name__ == "__main__":
    main()
