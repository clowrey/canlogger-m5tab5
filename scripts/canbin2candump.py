#!/usr/bin/env python3
"""Convert a Tab5 .canbin capture to candump (-l) log format.

candump line:  (<epoch>.<usec>) <iface> <ID>#<DATA>
Lets you diff a Tab5 capture against a SocketCAN/CANable capture frame-for-frame:

    python3 canbin2candump.py seg0000.canbin -o tab5.log -i can0
    # alongside: candump -l can0 (Linux) or scripts/canable_log.py (CANable, any OS)
    # then compare (ignoring the leading timestamps), or use tools/compare_logs.py:
    diff <(cut -d' ' -f2- tab5.log) <(cut -d' ' -f2- candump-*.log)

Record/header layout mirrors components/can_logger/can_logger.h (CANBIN01).
"""
import argparse
import struct
import sys

HDR = struct.Struct("<8sHHIQII")   # magic, ver, rec_size, bitrate, epoch_us, flags, rsvd
REC = struct.Struct("<QIBBBB8s")    # ts_us, id, dlc, flags, bus, rsvd, data

META = 1 << 29
EXT = 1 << 31
RTR = 1 << 30
EV = {1: "CAPTURE_START", 2: "FLASH_START", 3: "FLASH_END", 4: "OVERRUN_GAP", 5: "SEG_ROTATE"}


def read_records(path):
    with open(path, "rb") as f:
        hdr = f.read(HDR.size)
        if len(hdr) < HDR.size:
            sys.exit(f"{path}: only {len(hdr)} bytes, no CANBIN01 header (file not synced yet?)")
        magic, ver, rec_size, bitrate, epoch_us, _fl, _rs = HDR.unpack(hdr)
        if magic != b"CANBIN01":
            sys.exit(f"{path}: not a CANBIN01 file (magic={magic!r})")
        if rec_size != REC.size:
            sys.exit(f"{path}: rec_size {rec_size} != {REC.size}")
        yield ("hdr", bitrate, epoch_us)
        while True:
            raw = f.read(REC.size)
            if len(raw) < REC.size:
                break
            ts_us, idf, dlc, flags, bus, _r, data = REC.unpack(raw)
            yield ("rec", ts_us, idf, dlc, flags, bus, data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile")
    ap.add_argument("-o", "--out", default="-")
    ap.add_argument("-i", "--iface", default="can0")
    ap.add_argument("--keep-meta", action="store_true", help="emit markers as comments")
    args = ap.parse_args()

    out = sys.stdout if args.out == "-" else open(args.out, "w")
    epoch_us = 0
    n = 0
    for item in read_records(args.infile):
        if item[0] == "hdr":
            epoch_us = item[2]
            continue
        _, ts_us, idf, dlc, flags, bus, data = item
        if idf & META:
            if args.keep_meta:
                code = dlc
                out.write(f"# {EV.get(code, code)} id=0x{idf & 0x1FFFFFFF:X} ts={ts_us}\n")
            continue
        t = epoch_us + ts_us
        sec, usec = divmod(t, 1_000_000)
        cid = idf & 0x1FFFFFFF
        idstr = f"{cid:08X}" if (idf & EXT) else f"{cid:03X}"
        if idf & RTR:
            body = f"{idstr}#R"
        else:
            body = f"{idstr}#{data[:dlc].hex().upper()}"
        gap = "  ; GAP(frames lost before this)" if (flags & 1) else ""
        out.write(f"({sec}.{usec:06d}) {args.iface} {body}{gap}\n")
        n += 1
    if out is not sys.stdout:
        out.close()
    sys.stderr.write(f"{n} frames written\n")


if __name__ == "__main__":
    main()
