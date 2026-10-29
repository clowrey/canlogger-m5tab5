#!/usr/bin/env python3
"""Send numbered test frames from a gs_usb CANable, for checking the Tab5 capture
on a desk bus where nothing else transmits.

    python canable_testgen.py -n 2000                 # as fast as the bus allows
    python canable_testgen.py -n 500 --gap-ms 2       # paced

The CANable runs in external loopback: it drives the bus but does not need an
ACK (the Tab5 never ACKs, it has no TX pin), and it reads its own frames back.
Never use this on a car: it transmits.

With Elmue's Candlelight 2.5 the CANable sends about 2,500 frames/s this way
(--full: 8-byte frames, ~56 % of a 500 kbit/s bus); more than 32 frames in
flight makes it drop read-back copies. budgetcan_fw managed about 500 frames/s,
and more than 16 in flight stalled it.

Writes <prefix>_sent.log (what was queued) and <prefix>_rx.log (the CANable's
own copies), both with PC timestamps in candump -l format like
canbin2candump.py, so the Tab5 file can be diffed against them.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from canable_log import (CAN_EFF_FLAG, CAN_EFF_MASK, CAN_RTR_FLAG, FrameClock,  # noqa: E402
                         candump_line, find_gs_usb, set_bitrate)

GS_NONE_ECHO = 0xFFFFFFFF


class LoopbackSender:
    """Transmit on a gs_usb CANable in external loopback, logging what was sent
    and what the CANable read back."""

    def __init__(self, bitrate, prefix, iface="can0", window=8):
        from gs_usb.constants import GS_CAN_MODE_HW_TIMESTAMP, GS_CAN_MODE_LOOP_BACK
        from gs_usb.gs_usb_frame import GsUsbFrame

        self.Frame = GsUsbFrame
        self.dev = find_gs_usb()
        if self.dev is None:
            sys.exit("no gs_usb CANable found")
        if not self.dev.device_capability.feature & GS_CAN_MODE_LOOP_BACK:
            sys.exit("firmware has no loopback mode")
        try:
            self.dev.stop()
        except Exception:
            pass
        if not set_bitrate(self.dev, bitrate):
            sys.exit(f"bitrate {bitrate} not supported")
        self.dev.start(GS_CAN_MODE_LOOP_BACK | GS_CAN_MODE_HW_TIMESTAMP)
        self.iface = iface
        self.sent_f = open(f"{prefix}_sent.log", "w", newline="\n")
        self.rx_f = open(f"{prefix}_rx.log", "w", newline="\n")
        self.sent_name, self.rx_name = self.sent_f.name, self.rx_f.name
        self.free = list(range(window))       # echo ids not in flight
        self.clock = FrameClock(True)
        self.nsent = self.nrx = self.necho = 0
        self.rx = GsUsbFrame()
        self.t0 = time.time()

    def _poll(self, timeout_ms):
        rx = self.rx
        if not self.dev.read(rx, timeout_ms):
            return False
        if rx.echo_id != GS_NONE_ECHO:
            self.free.append(rx.echo_id)      # our frame left the controller
            self.necho += 1
            return True
        self.rx_f.write(candump_line(self.clock.stamp(rx.timestamp_us), self.iface,
                                     rx.can_id & CAN_EFF_MASK, bool(rx.can_id & CAN_EFF_FLAG),
                                     bool(rx.can_id & CAN_RTR_FLAG),
                                     bytes(rx.data[:min(rx.can_dlc, 8)])))
        self.nrx += 1
        return True

    def send(self, can_id, ext=False, rtr=False, data=b""):
        while not self.free:
            if not self._poll(1000) and not self.free:
                sys.exit(f"no TX echo for 1 s after {self.nsent} frames: "
                         "is the bus wired and powered?")
        f = self.Frame(can_id | (CAN_EFF_FLAG if ext else 0) | (CAN_RTR_FLAG if rtr else 0),
                       bytes(data))
        if rtr:
            f.can_dlc = 0
        f.echo_id = self.free.pop()
        self.dev.send(f)
        self.sent_f.write(candump_line(time.time(), self.iface, can_id, ext, rtr, bytes(data)))
        self.nsent += 1

    def close(self):
        deadline = time.time() + 1.0
        while time.time() < deadline and (self.necho < self.nsent or self.nrx < self.nsent):
            self._poll(100)
        self.dev.stop()
        self.sent_f.close()
        self.rx_f.close()
        dt = time.time() - self.t0
        print(f"sent {self.nsent}, TX echoes {self.necho}, read back {self.nrx} in {dt:.1f} s "
              f"({self.nsent / dt:.0f} frames/s) -> {self.sent_name}, {self.rx_name}")


def make_frame(i, full=False):
    """Frame i: (id, ext, rtr, data). The first data bytes carry i. full: every
    frame a standard id with 8 data bytes, like ISO-TP consecutive frames."""
    ext = not full and i % 10 == 9
    rtr = not full and i % 97 == 50
    can_id = (0x18DA00F1 + (i % 7)) if ext else 0x100 + (i % 0x400)
    dlc = 8 if full else i % 9
    data = b"" if rtr else (i.to_bytes(4, "big") + bytes([0xA5, 0x5A, 0xC3, 0x3C]))[:dlc]
    return can_id, ext, rtr, data


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-n", "--count", type=int, default=1000)
    ap.add_argument("-b", "--bitrate", type=int, default=500000)
    ap.add_argument("--gap-ms", type=float, default=0, help="pause between frames (0 = bus speed)")
    ap.add_argument("--window", type=int, default=8, help="frames in flight on USB")
    ap.add_argument("--full", action="store_true", help="8-byte standard frames only (worst case)")
    ap.add_argument("-p", "--prefix", default="testgen")
    ap.add_argument("-i", "--iface", default="can0")
    args = ap.parse_args()

    tx = LoopbackSender(args.bitrate, args.prefix, args.iface, args.window)
    try:
        for i in range(args.count):
            tx.send(*make_frame(i, args.full))
            if args.gap_ms:
                time.sleep(args.gap_ms / 1000)
    finally:
        tx.close()


if __name__ == "__main__":
    main()
