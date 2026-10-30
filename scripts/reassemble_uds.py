#!/usr/bin/env python3
"""Reconstruct a firmware image from a captured UDS-over-CAN flash session.

Reads a Tab5 .canbin capture (the segment holding the session), reassembles
ISO-TP messages on the tester->ECU id, and concatenates the payloads of every
TransferData (0x36) request in block-sequence order into a raw .bin.

    python3 reassemble_uds.py seg0012.canbin --tx-id 0x602 -o image.bin

What it reports:
  * RequestDownload (0x34): dataFormatId, address, uncompressed size
  * each TransferData (0x36) block seq + length
  * RequestTransferExit (0x37)
This is a first-cut reassembler: it assumes normal-addressing ISO-TP (classic,
or CAN FD with up to 64-byte frames) and that the tester's requests are on a
single id. Set --tx-id to the tester->ECU id of the diag channel you captured,
and --bus if the same id exists on both channels. Validate the result's size
against the 0x34 announced size and the image's own header/CRC before trusting it.
"""
import argparse
import sys

import canbin


def frames(path, want_id, bus=None):
    """Yield (ts_us, data_bytes) for data frames on want_id (None = any)."""
    it = canbin.read(path)
    next(it)  # header
    for fr in it:
        if fr.meta or fr.rtr or (bus is not None and fr.bus != bus):
            continue
        if want_id is None or fr.id == want_id:
            yield fr.ts_us, fr.data


def isotp_messages(frame_iter):
    """Reassemble ISO-TP (normal addressing). Yields complete UDS payloads."""
    buf = bytearray()
    remaining = 0
    assembling = False
    for ts, d in frame_iter:
        if not d:
            continue
        pci = d[0] >> 4
        if pci == 0x0:  # single frame
            length = d[0] & 0x0F
            if length == 0 and len(d) >= 2:  # escape: CAN FD single frame over 8 bytes
                length = d[1]
                payload = d[2:2 + length]
            else:
                payload = d[1:1 + length]
            yield bytes(payload)
            assembling = False
        elif pci == 0x1:  # first frame
            length = ((d[0] & 0x0F) << 8) | d[1]
            if length == 0 and len(d) >= 6:  # escape: 32-bit length (over 4095 bytes)
                length = int.from_bytes(d[2:6], "big")
                buf = bytearray(d[6:])
            else:
                buf = bytearray(d[2:])
            remaining = length - len(buf)
            assembling = True
        elif pci == 0x2:  # consecutive frame
            if not assembling:
                continue
            chunk = d[1:]
            take = min(remaining, len(chunk))
            buf += chunk[:take]
            remaining -= take
            if remaining <= 0:
                yield bytes(buf)
                assembling = False
        # pci == 0x3 (flow control) comes from the ECU side; ignore here


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile")
    ap.add_argument("--tx-id", type=lambda x: int(x, 0), required=True,
                    help="tester->ECU CAN id carrying 0x34/0x36 (e.g. 0x602)")
    ap.add_argument("--bus", type=int, choices=(0, 1),
                    help="only frames from this bus (0 = Port A, 1 = FD channel)")
    ap.add_argument("-o", "--out", default="image.bin")
    args = ap.parse_args()

    blocks = {}           # seq -> payload bytes
    order = []            # seq order as seen
    announced_size = None
    saw_exit = False

    for msg in isotp_messages(frames(args.infile, args.tx_id, args.bus)):
        if not msg:
            continue
        svc = msg[0]
        if svc == 0x34 and len(msg) >= 3:
            dfi = msg[1]
            alfi = msg[2]
            size_len = alfi >> 4
            addr_len = alfi & 0x0F
            p = 3
            addr = int.from_bytes(msg[p:p + addr_len], "big"); p += addr_len
            size = int.from_bytes(msg[p:p + size_len], "big"); p += size_len
            announced_size = size
            print(f"[0x34] RequestDownload dfi=0x{dfi:02X} addr=0x{addr:X} size={size} "
                  f"({size/1024:.1f} KB)")
        elif svc == 0x36 and len(msg) >= 2:
            seq = msg[1]
            payload = msg[2:]
            blocks[(len(order), seq)] = payload  # keep arrival order unique
            order.append((seq, len(payload)))
            print(f"[0x36] TransferData seq=0x{seq:02X} len={len(payload)}")
        elif svc == 0x37:
            saw_exit = True
            print("[0x37] RequestTransferExit")

    # concatenate in arrival order (UDS TransferData is strictly sequential)
    image = bytearray()
    for (idx, seq), payload in blocks.items():
        image += payload
    with open(args.out, "wb") as f:
        f.write(image)

    print("-" * 60)
    print(f"blocks: {len(order)}   reassembled: {len(image)} bytes -> {args.out}")
    if announced_size is not None:
        print(f"announced (0x34): {announced_size} bytes  "
              f"{'MATCH' if announced_size == len(image) else 'MISMATCH'}")
    if not saw_exit:
        print("WARNING: no 0x37 RequestTransferExit seen — capture may be truncated")


if __name__ == "__main__":
    main()
