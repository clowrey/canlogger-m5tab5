#!/usr/bin/env python3
"""Convert a Tab5 .canbin capture to candump (-l) log format.

candump line:  (<epoch>.<usec>) <iface> <ID>#<DATA>     classic
               (<epoch>.<usec>) <iface> <ID>##<F><DATA>  CAN FD (F: 1 = BRS, 2 = ESI)
Lets you diff a Tab5 capture against a SocketCAN/CANable capture frame-for-frame:

    python3 canbin2candump.py seg0000.canbin -o tab5.log -i can0
    # alongside: candump -l can0 (Linux) or scripts/canable_log.py (CANable, any OS)
    # then compare (ignoring the leading timestamps), or use tools/compare_logs.py:
    diff <(cut -d' ' -f2- tab5.log) <(cut -d' ' -f2- candump-*.log)

Bus 0 (Port A) is written as --iface, bus 1 (the FD channel, CANBIN02) as
--iface1; --bus keeps only one of them. Lines are in file order: within a bus
that is time order, but the two buses are interleaved by arrival (sort by the
timestamp to merge them).
"""
import argparse
import sys

import canbin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile")
    ap.add_argument("-o", "--out", default="-")
    ap.add_argument("-i", "--iface", default="can0", help="interface name for bus 0")
    ap.add_argument("--iface1", default="can1", help="interface name for bus 1 (FD channel)")
    ap.add_argument("--bus", type=int, choices=(0, 1), help="only this bus")
    ap.add_argument("--keep-meta", action="store_true", help="emit markers as comments")
    args = ap.parse_args()

    out = sys.stdout if args.out == "-" else open(args.out, "w")
    it = canbin.read(args.infile)
    hdr = next(it)
    n = 0
    for fr in it:
        if args.bus is not None and fr.bus != args.bus:
            continue
        if fr.meta:
            if args.keep_meta:
                out.write(f"# {canbin.EV.get(fr.dlc, fr.dlc)} id=0x{fr.id:X} ts={fr.ts_us}\n")
            continue
        sec, usec = divmod(hdr.epoch_us + fr.ts_us, 1_000_000)
        iface = args.iface1 if fr.bus == 1 else args.iface
        gap = "  ; GAP(frames lost before this)" if fr.gap else ""
        out.write(f"({sec}.{usec:06d}) {iface} {canbin.candump_body(fr)}{gap}\n")
        n += 1
    if out is not sys.stdout:
        out.close()
    sys.stderr.write(f"{n} frames written\n")


if __name__ == "__main__":
    main()
