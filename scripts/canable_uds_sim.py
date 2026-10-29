#!/usr/bin/env python3
"""Play a synthetic UDS-over-CAN flash session from a gs_usb CANable, for
checking the Tab5's flash-file carving and reassemble_uds.py on a desk bus.

    python canable_uds_sim.py --size 20480 -p uds

Both sides are sent by the CANable (external loopback, no ACK needed; see
canable_testgen.py): tester requests on --tx-id, ECU replies and ISO-TP flow
control on --rx-id, plus background traffic on other ids. Never use this on a
car: it transmits.

Sequence: background, 0x10 02 / 0x27 seed-key, 0x34 RequestDownload (multi-frame),
0x36 TransferData blocks, 0x37 RequestTransferExit, then background for longer
than the Tab5's post-roll. Writes <prefix>_image.bin (the image that was sent)
and the logs from canable_testgen.LoopbackSender.
"""
import argparse
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from canable_testgen import LoopbackSender  # noqa: E402

PAD = 0x55


class Bus:
    """LoopbackSender plus background frames every few frames."""

    def __init__(self, tx, every):
        self.tx, self.every, self.n, self.bg = tx, every, 0, 0

    def send(self, can_id, data):
        self.tx.send(can_id, data=bytes(data))
        self.n += 1
        if self.every and self.n % self.every == 0:
            self.background()

    def background(self):
        i = self.bg
        self.bg += 1
        if i % 4 == 3:
            self.tx.send(0x18FF5000 + (i % 3), ext=True, data=i.to_bytes(4, "big"))
        else:
            self.tx.send(0x200 + (i % 16), data=i.to_bytes(4, "big") + b"\x11\x22\x33\x44")

    def idle(self, seconds, gap=0.01):
        end = time.time() + seconds
        while time.time() < end:
            self.background()
            time.sleep(gap)


def pad8(b):
    return bytes(b) + bytes([PAD]) * (8 - len(b))


def isotp(bus, tx_id, rx_id, payload):
    """Send one ISO-TP message tx_id -> rx_id, with the receiver's flow control."""
    if len(payload) <= 7:
        bus.send(tx_id, pad8([len(payload)] + list(payload)))
        return
    n = len(payload)
    bus.send(tx_id, [0x10 | (n >> 8), n & 0xFF] + list(payload[:6]))
    bus.send(rx_id, pad8([0x30, 0x00, 0x00]))          # CTS, no block limit, no STmin
    sn, p = 1, 6
    while p < n:
        bus.send(tx_id, pad8([0x20 | sn] + list(payload[p:p + 7])))
        sn, p = (sn + 1) & 0x0F, p + 7


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--size", type=int, default=20480, help="image bytes")
    ap.add_argument("--block", type=int, default=1024, help="data bytes per 0x36")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x00010000)
    ap.add_argument("--tx-id", type=lambda x: int(x, 0), default=0x602, help="tester -> ECU")
    ap.add_argument("--rx-id", type=lambda x: int(x, 0), default=0x612, help="ECU -> tester")
    ap.add_argument("--pre-s", type=float, default=2.0, help="background before the session")
    ap.add_argument("--post-s", type=float, default=3.0, help="background after 0x37")
    ap.add_argument("--bg-every", type=int, default=10, help="background frame every N session frames")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("-b", "--bitrate", type=int, default=500000)
    ap.add_argument("-p", "--prefix", default="uds")
    args = ap.parse_args()

    image = random.Random(args.seed).randbytes(args.size)
    with open(f"{args.prefix}_image.bin", "wb") as f:
        f.write(image)

    tx = LoopbackSender(args.bitrate, args.prefix)
    bus = Bus(tx, args.bg_every)
    T, E = args.tx_id, args.rx_id
    try:
        bus.idle(args.pre_s)
        isotp(bus, T, E, [0x10, 0x02])                         # programming session
        isotp(bus, E, T, [0x50, 0x02, 0x00, 0x19, 0x01, 0xF4])
        isotp(bus, T, E, [0x27, 0x01])                         # request seed
        isotp(bus, E, T, [0x67, 0x01, 0xDE, 0xAD, 0xBE, 0xEF])
        isotp(bus, T, E, [0x27, 0x02, 0x21, 0x52, 0x41, 0x10])  # send key
        isotp(bus, E, T, [0x67, 0x02])
        bus.idle(0.3)
        req = [0x34, 0x00, 0x44] + list(args.addr.to_bytes(4, "big")) + list(args.size.to_bytes(4, "big"))
        isotp(bus, T, E, req)                                  # RequestDownload
        max_len = args.block + 2
        isotp(bus, E, T, [0x74, 0x20, max_len >> 8, max_len & 0xFF])
        seq = 1
        for off in range(0, args.size, args.block):
            isotp(bus, T, E, [0x36, seq] + list(image[off:off + args.block]))
            isotp(bus, E, T, [0x76, seq])
            seq = (seq + 1) & 0xFF
        isotp(bus, T, E, [0x37])                               # RequestTransferExit
        isotp(bus, E, T, [0x77])
        bus.idle(args.post_s)
    finally:
        tx.close()
    print(f"image {args.size} bytes in {(args.size + args.block - 1) // args.block} blocks "
          f"-> {args.prefix}_image.bin")


if __name__ == "__main__":
    main()
