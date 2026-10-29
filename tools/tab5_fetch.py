#!/usr/bin/env python3
"""List or download capture files from the Tab5's web server.

    python tools/tab5_fetch.py <tab5-ip> --list
    python tools/tab5_fetch.py <tab5-ip> seg                 # newest segNNNN.canbin
    python tools/tab5_fetch.py <tab5-ip> flash -o captures   # newest flash_<boot>_<ms>.canbin
    python tools/tab5_fetch.py <tab5-ip> seg0012.canbin      # a named file

The web_server login is read from esphome/secrets.yaml (web_server_username /
web_server_password) and never printed. Large files are slow over the Tab5's
Wi-Fi; for big captures, use Safe shutdown and copy the SD card instead.
Needs: pip install requests pyyaml
"""
import argparse
import os
import re
import sys

import requests
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SECRETS = os.path.join(HERE, "..", "esphome", "secrets.yaml")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("host")
    ap.add_argument("what", nargs="?", default="seg", help="seg, flash or a file name")
    ap.add_argument("-o", "--outdir", default=".")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--secrets", default=DEFAULT_SECRETS)
    args = ap.parse_args()

    s = yaml.safe_load(open(args.secrets))
    auth = (s["web_server_username"], s["web_server_password"])
    html = requests.get(f"http://{args.host}/files", auth=auth, timeout=20)
    html.raise_for_status()
    segs = sorted(set(re.findall(r"seg\d+\.canbin", html.text)), key=lambda n: int(n[3:-7]))
    flashes = sorted(set(re.findall(r"flash_\d+_\d+\.canbin", html.text)),
                     key=lambda n: tuple(int(x) for x in n[6:-7].split("_")))
    if args.list:
        print("\n".join(segs + flashes))
        return
    if args.what == "seg":
        names = segs
    elif args.what == "flash":
        names = flashes
    else:
        names = [args.what] if args.what in segs + flashes else []
    if not names:
        sys.exit(f"nothing matching {args.what!r} on the Tab5")
    name = names[-1]
    r = requests.get(f"http://{args.host}/dl?f={name}", auth=auth, timeout=1800)
    r.raise_for_status()
    os.makedirs(args.outdir, exist_ok=True)
    path = os.path.join(args.outdir, name)
    with open(path, "wb") as f:
        f.write(r.content)
    print(path, len(r.content))


if __name__ == "__main__":
    main()
