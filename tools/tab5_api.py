#!/usr/bin/env python3
"""Read the Tab5 logger's state or press one of its buttons over the ESPHome API.

    python tools/tab5_api.py states <tab5-ip>
    python tools/tab5_api.py states <tab5-ip> --watch 30      # print rate/load as they change
    python tools/tab5_api.py press  <tab5-ip> "Safe shutdown (close SD files)"

The API key is read from esphome/secrets.yaml (api_encryption_key) and never printed.
Needs: pip install aioesphomeapi pyyaml
"""
import argparse
import asyncio
import os
import time

import yaml
from aioesphomeapi import APIClient, ButtonInfo

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SECRETS = os.path.join(HERE, "..", "esphome", "secrets.yaml")
SUMMARY = ("CAN frames captured", "CAN frame rate", "CAN bus load", "CAN overruns", "CAN bus errors",
           "Capture MB written", "SD OK", "Capture valid (0 overruns)", "Flash session active",
           "Safe to power off", "WiFi Signal")
LIVE = ("CAN frame rate", "CAN bus load")


async def run(args):
    key = yaml.safe_load(open(args.secrets))["api_encryption_key"]
    cli = APIClient(args.host, 6053, None, noise_psk=key)
    await cli.connect(login=True)
    entities, _ = await cli.list_entities_services()
    names = {e.key: e.name for e in entities}
    states = {}
    t0 = time.time()

    def on_state(s):
        name = names.get(s.key, s.key)
        states[name] = getattr(s, "state", None)
        if args.cmd == "states" and args.watch and name in LIVE:
            print(f"{time.time() - t0:5.1f}s  {name}: {states[name]:.0f}", flush=True)

    cli.subscribe_states(on_state)
    await asyncio.sleep(2)
    if args.cmd == "press":
        print("before:", {k: states.get(k) for k in SUMMARY})
        btn = next((e for e in entities if isinstance(e, ButtonInfo) and e.name == args.button), None)
        if btn is None:
            raise SystemExit("no button named " + repr(args.button) + "; buttons: " +
                             ", ".join(e.name for e in entities if isinstance(e, ButtonInfo)))
        cli.button_command(btn.key)
        print(f"pressed {args.button!r}")
        await asyncio.sleep(args.wait)
        print("after: ", {k: states.get(k) for k in SUMMARY})
    else:
        await asyncio.sleep(max(4, args.watch))
        print({k: states.get(k) for k in SUMMARY})
    await cli.disconnect()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--secrets", default=DEFAULT_SECRETS)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("states")
    s.add_argument("host")
    s.add_argument("--watch", type=float, default=0, help="seconds to stream rate/load changes")
    p = sub.add_parser("press")
    p.add_argument("host")
    p.add_argument("button")
    p.add_argument("--wait", type=float, default=5, help="seconds to wait before reading states again")
    asyncio.run(run(ap.parse_args()))


if __name__ == "__main__":
    main()
