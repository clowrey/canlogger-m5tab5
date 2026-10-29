#!/usr/bin/env python3
"""Compare candump logs frame by frame (timestamps ignored).

    # every frame the CANable sent must be in the Tab5 capture, in order
    python tools/compare_logs.py exact tab5.log run1_sent.log run2_sent.log

    # a carved flash file must be one unbroken stretch of what was sent, with the
    # pre-roll before 0x34 and the post-roll after 0x37 measured
    python tools/compare_logs.py slice flash.log uds_sent.log --tx-id 0x602

Logs are candump -l lines "(ts) iface ID#DATA" as written by canbin2candump.py,
canable_log.py and the canable_* test senders; '#' lines (meta records) are
skipped. In exact mode, frames in the capture that match --ignore (e.g. other
traffic sent outside the logged runs) are left out before comparing.
"""
import argparse
import sys


def read(path):
    """[(ts, 'ID#DATA')] for every frame line."""
    out = []
    for line in open(path):
        if line.startswith("("):
            ts, _iface, body = line.split(" ", 2)
            out.append((float(ts[1:-1]), body.strip()))
    return out


def exact(args):
    got = [b for _, b in read(args.capture) if b not in set(args.ignore)]
    want = [b for p in args.sent for _, b in read(p)]
    print(f"capture {len(got)} frames, sent {len(want)}")
    if got == want:
        ext = sum(1 for f in want if len(f.split("#")[0]) == 8)
        rtr = sum(1 for f in want if f.endswith("#R"))
        lens = sorted({len(f.split("#")[1]) // 2 for f in want if not f.endswith("#R")})
        print(f"IDENTICAL: same frames in the same order ({ext} extended ids, {rtr} remote "
              f"frames, data lengths {lens})")
        return 0
    for i, (a, b) in enumerate(zip(got, want)):
        if a != b:
            print(f"DIFFERENT at frame {i}: capture {a}, sent {b}")
            return 1
    print(f"DIFFERENT: one log is a prefix of the other ({len(got)} vs {len(want)})")
    return 1


def slice_(args):
    flash = read(args.flash)
    sent = read(args.sent)
    body = [b for _, b in flash]
    want = [b for _, b in sent]
    tx = f"{args.tx_id:03X}#"
    starts = [i for i, b in enumerate(body) if b.startswith(tx) and service(b) == 0x34]
    print(f"flash file {len(body)} frames; 0x34 trigger frames in it: {len(starts)}")
    for k in range(len(want) - len(body) + 1):
        if want[k:k + len(body)] == body:
            end = k + len(body) - 1
            trig = next(i for i, b in enumerate(want) if b.startswith(tx) and service(b) == 0x34)
            done = next(i for i, b in enumerate(want) if b.startswith(tx) and service(b) == 0x37)
            print(f"CONTIGUOUS: sent frames [{k}..{end}] of {len(want)} "
                  f"({k} before and {len(want) - 1 - end} after are outside the file)")
            print(f"starts {sent[trig][0] - sent[k][0]:.2f} s before 0x34, "
                  f"ends {sent[end][0] - sent[done][0]:.2f} s after 0x37 (sender timestamps)")
            return 0 if len(starts) == 1 else 1
    print("NOT a contiguous stretch of what was sent")
    return 1


def service(body):
    """UDS service byte of an ISO-TP single or first frame, else None."""
    data = bytes.fromhex(body.split("#")[1]) if not body.endswith("#R") else b""
    if len(data) >= 2 and data[0] >> 4 == 0:
        return data[1]
    if len(data) >= 3 and data[0] >> 4 == 1:
        return data[2]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="mode", required=True)
    e = sub.add_parser("exact")
    e.add_argument("capture")
    e.add_argument("sent", nargs="+")
    e.add_argument("--ignore", nargs="*", default=[], metavar="ID#DATA")
    s = sub.add_parser("slice")
    s.add_argument("flash")
    s.add_argument("sent")
    s.add_argument("--tx-id", type=lambda x: int(x, 0), default=0x602)
    args = ap.parse_args()
    sys.exit(exact(args) if args.mode == "exact" else slice_(args))


if __name__ == "__main__":
    main()
