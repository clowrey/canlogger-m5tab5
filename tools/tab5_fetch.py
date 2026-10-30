#!/usr/bin/env python3
"""List or download capture files from the Tab5's web server.

    python tools/tab5_fetch.py <tab5-ip> --list
    python tools/tab5_fetch.py <tab5-ip> seg -o captures     # newest segNNNN.canbin
    python tools/tab5_fetch.py <tab5-ip> seg0012.canbin      # a named file

The web_server login is read from esphome/secrets.yaml (web_server_username /
web_server_password) and never printed. A download that the Wi-Fi drops is
resumed where it stopped (HTTP Range; the file grows as NAME.part until it's
complete), up to --retries times; running the command again also resumes.
Large files are still slow over the Tab5's Wi-Fi; for big captures, use Safe
shutdown and copy the SD card instead.
Needs: pip install requests pyyaml
"""
import argparse
import os
import re
import sys
import time

import requests
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SECRETS = os.path.join(HERE, "..", "esphome", "secrets.yaml")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("host")
    ap.add_argument("what", nargs="?", default="seg", help="seg (the newest segment) or a file name")
    ap.add_argument("-o", "--outdir", default=".")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--secrets", default=DEFAULT_SECRETS)
    ap.add_argument("--retries", type=int, default=20, help="resume attempts after a drop")
    args = ap.parse_args()

    s = yaml.safe_load(open(args.secrets))
    auth = (s["web_server_username"], s["web_server_password"])
    html = requests.get(f"http://{args.host}/files", auth=auth, timeout=20)
    html.raise_for_status()
    segs = sorted(set(re.findall(r"seg\d+\.canbin", html.text)), key=lambda n: int(n[3:-7]))
    # flash_*.canbin: files carved by older firmware, still listed and downloadable
    flashes = sorted(set(re.findall(r"flash_\d+_\d+\.canbin", html.text)))
    if args.list:
        print("\n".join(segs + flashes))
        return
    if args.what == "seg":
        names = segs
    else:
        names = [args.what] if args.what in segs + flashes else []
    if not names:
        sys.exit(f"nothing matching {args.what!r} on the Tab5")
    name = names[-1]
    os.makedirs(args.outdir, exist_ok=True)
    path = os.path.join(args.outdir, name)
    n = fetch(f"http://{args.host}/dl?f={name}", auth, path, args.retries)
    print(path, n)


def fetch(url, auth, path, retries):
    """Download to path + '.part', resuming with a Range request after a drop,
    until it has the size the Tab5 reports (X-File-Size, or Content-Range)."""
    part = path + ".part"
    have = os.path.getsize(part) if os.path.exists(part) else 0
    total = None
    for attempt in range(retries + 1):
        try:
            hdrs = {"Range": f"bytes={have}-"} if have else {}
            with requests.get(url, auth=auth, headers=hdrs, stream=True, timeout=30) as r:
                if r.status_code == 416:  # already complete
                    break
                r.raise_for_status()
                if have and r.status_code != 206:
                    sys.exit("the Tab5 ignored the Range request (older firmware): "
                             f"delete {part} and download again, or copy the SD card")
                if "X-File-Size" in r.headers:
                    total = int(r.headers["X-File-Size"])
                with open(part, "ab" if have else "wb") as f:
                    for chunk in r.iter_content(65536):
                        f.write(chunk)
                        have += len(chunk)
            if total is None or have >= total:
                break
        except (requests.ConnectionError, requests.Timeout,
                requests.exceptions.ChunkedEncodingError) as e:
            have = os.path.getsize(part) if os.path.exists(part) else 0
            pct = f" of {total} ({100 * have / total:.0f} %)" if total else ""
            print(f"  dropped at {have} bytes{pct}: {type(e).__name__}; "
                  f"retry {attempt + 1}/{retries}", file=sys.stderr)
            time.sleep(3)
    else:
        sys.exit(f"gave up after {retries} retries; run again to resume from {have} bytes")
    if total is not None and have < total:
        sys.exit(f"incomplete: {have} of {total} bytes; run again to resume")
    os.replace(part, path)
    return have


if __name__ == "__main__":
    main()
