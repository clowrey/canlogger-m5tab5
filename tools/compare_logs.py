#!/usr/bin/env python3
"""Compare candump logs frame by frame (timestamps ignored).

    # every frame the CANable sent must be in the Tab5 capture, in order
    python tools/compare_logs.py exact tab5.log run1_sent.log run2_sent.log

    # live bus: every frame another logger saw (e.g. canable_log.py) must be in
    # the Tab5 capture, in order; frames only the capture has are listed
    python tools/compare_logs.py subset tab5.log canable.log

Logs are candump -l lines "(ts) iface ID#DATA" as written by canbin2candump.py,
canable_log.py and the canable_* test senders; '#' lines (meta records) are
skipped. In exact mode, frames in the capture that match --ignore (e.g. other
traffic sent outside the logged runs) are left out before comparing. In subset
mode the capture may start earlier and end later than the reference; the
reference's first frames locate it in the capture.
"""
import argparse
import sys


def read(path, iface=None):
    """[(ts, 'ID#DATA')] for every frame line (only `iface`'s, if given)."""
    out = []
    for line in open(path):
        if line.startswith("("):
            ts, if_, body = line.split(" ", 2)
            if iface is None or if_ == iface:
                out.append((float(ts[1:-1]), body.strip()))
    return out


def data_len(body):
    """Data bytes of 'ID#DATA' or CAN FD 'ID##<flags>DATA'."""
    d = body.split(" ")[0].split("#", 1)[1]
    return len(d[2:] if d.startswith("#") else d) // 2


def exact(args):
    got = [b for _, b in read(args.capture, args.iface) if b not in set(args.ignore)]
    want = [b for p in args.sent for _, b in read(p)]
    print(f"capture {len(got)} frames, sent {len(want)}")
    if got == want:
        ext = sum(1 for f in want if len(f.split("#")[0]) == 8)
        rtr = sum(1 for f in want if f.endswith("#R"))
        lens = sorted({data_len(f) for f in want if not f.endswith("#R")})
        print(f"IDENTICAL: same frames in the same order ({ext} extended ids, {rtr} remote "
              f"frames, data lengths {lens})")
        return 0
    for i, (a, b) in enumerate(zip(got, want)):
        if a != b:
            print(f"DIFFERENT at frame {i}: capture {a}, sent {b}")
            return 1
    print(f"DIFFERENT: one log is a prefix of the other ({len(got)} vs {len(want)})")
    return 1


def subset(args):
    got = read(args.capture, args.iface)
    ref = read(args.reference)
    body = [b for _, b in got]
    want = [b for _, b in ref]
    k = 16
    i = next((x for x in range(len(body) - k + 1) if body[x:x + k] == want[:k]), None)
    if i is None:
        print(f"the reference's first {k} frames are not in the capture")
        return 1
    start, missing, extra = i, 0, []
    for j, b in enumerate(want):
        x = i
        while x < len(body) and x - i < args.window and body[x] != b:
            x += 1
        if x >= len(body) or body[x] != b:
            missing += 1
            if missing <= 5:
                print(f"  reference frame {j} not in the capture: {b}")
            continue
        if x > i:
            extra.append((x - i, ref[j][0] - ref[0][0]))
        i = x + 1
    print(f"reference {len(want)} frames; capture {i - start} frames over the same stretch")
    print(f"reference frames missing from the capture: {missing}")
    print(f"frames only in the capture: {sum(n for n, _ in extra)} in {len(extra)} bursts"
          + (" (at s into the reference: " + ", ".join(f"{n} @ {t:.1f}" for n, t in extra[:20])
             + (" ..." if len(extra) > 20 else "") + ")" if extra else ""))
    if missing == 0:
        print("SUBSET: every reference frame is in the capture, same order")
    return 0 if missing == 0 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--iface", help="use only this interface's lines from the capture "
                    "(e.g. can1 for the FD channel of a CANBIN02 capture)")
    sub = ap.add_subparsers(dest="mode", required=True)
    e = sub.add_parser("exact")
    e.add_argument("capture")
    e.add_argument("sent", nargs="+")
    e.add_argument("--ignore", nargs="*", default=[], metavar="ID#DATA")
    s = sub.add_parser("subset")
    s.add_argument("capture")
    s.add_argument("reference")
    s.add_argument("--window", type=int, default=5000,
                   help="how far ahead in the capture to look for the next reference frame")
    args = ap.parse_args()
    sys.exit(exact(args) if args.mode == "exact" else subset(args))


if __name__ == "__main__":
    main()
