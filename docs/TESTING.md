# Testing

Desk tests with the Tab5 and a CANable 2.0 wired to the same two CAN wires
(CAN-H, CAN-L, plus termination). Nothing else is on the bus.

**These senders transmit. Never run them on a car.** On a car, only
`scripts/canable_log.py` (listen-only) belongs on the bus.

## Setup

```bash
pip install gs_usb libusb-package pyserial aioesphomeapi requests pyyaml
```

- CANable with gs_usb firmware that offers listen-only and loopback: Elmue's
  Candlelight 2.5 with the board patch in [canable-fw](../canable-fw/README.md).
- `esphome/secrets.yaml` filled in: the tools read the API key and the
  web_server login from it.

## Why loopback

The Tab5 has no TX pin, so it never ACKs. A frame that nobody ACKs gets an
error flag from its sender before the end of the frame, and every receiver,
including the Tab5, throws it away. The test senders put the CANable in
**external loopback**: it drives the bus but doesn't need an ACK, and it reads
its own frames back. Each run writes `<prefix>_sent.log` (what was sent) and
`<prefix>_rx.log` (what the CANable read back), both in candump format.

## 1. Plain traffic

```bash
python tools/tab5_api.py states <tab5-ip>                     # note "CAN frames captured"
python scripts/canable_testgen.py -n 20000 -p run1            # mixed: std/ext ids, RTR, 0-8 bytes
python scripts/canable_testgen.py -n 40000 -p run2 --full     # 8-byte frames only (worst case)
python tools/tab5_api.py states <tab5-ip>                     # count must rise by exactly 60,000, overruns 0
```

Frame-by-frame check against the Tab5's segment file:

```bash
python tools/tab5_fetch.py <tab5-ip> seg -o captures
python scripts/canbin2candump.py captures/segNNNN.canbin -o tab5.log -i can0
python tools/compare_logs.py exact tab5.log run1_sent.log run2_sent.log
```

`exact` passes only when the capture holds exactly the sent frames in the same
order. If other frames went out between the runs, list them with `--ignore`.

Watch the live rate while a run is going:

```bash
python tools/tab5_api.py states <tab5-ip> --watch 30
```

## 2. A whole UDS flash session

```bash
python scripts/canable_uds_sim.py -p uds --pre-s 8
python tools/tab5_fetch.py <tab5-ip> flash -o captures
python scripts/reassemble_uds.py captures/flash_*.canbin --tx-id 0x602 -o uds_rebuilt.bin
python scripts/canbin2candump.py captures/flash_*.canbin -o flash.log -i can0 --keep-meta
python tools/compare_logs.py slice flash.log uds_sent.log --tx-id 0x602
```

`canable_uds_sim.py` plays both sides with background traffic throughout:
`0x10 02`, `0x27` seed/key, a multi-frame `0x34` RequestDownload, `0x36`
TransferData blocks with ISO-TP flow control from the ECU id (`0x612`), `0x37`,
then traffic for longer than the post-roll. It saves the image it sent as
`<prefix>_image.bin`. Pass when:

- `reassemble_uds.py` says MATCH and the rebuilt image's SHA-256 equals
  `uds_image.bin`'s;
- `compare_logs.py slice` says CONTIGUOUS, with one `0x34` frame, starting about
  `preroll_ms` before `0x34` and ending about `postroll_ms` after `0x37`.

## 3. Screen, details and rate under load

Run section 1's `--full` sender with `states --watch` and look at the screen:
frames/s and bus load update once a second, overruns stay 0. Tap the header
for the details page. `tools/tab5_api.py states <tab5-ip> --all` prints every
entity (unique IDs, busiest IDs, buffers, SD, UDS progress, health).

Only one program can hold the CANable at a time; a second one stops with
"CANable is busy".

## Results (2026-10-28, ESPHome 2026.7.2, firmware from this repo)

| Test | Result |
|---|---|
| 20,200 mixed frames (2,020 extended, 208 RTR, lengths 0-8), ~500 frames/s | Captured, 0 overruns; `exact`: identical order and content |
| 80,000 frames incl. 40,000 eight-byte at 2,441 frames/s (~56 % bus) | All counted, 0 overruns, 0 bus errors |
| 60,000 eight-byte at 2,555-2,766 frames/s (57-61 % bus), screen redrawing | All counted, 0 overruns; rate and load shown live |
| Synthetic flash, 20 KB image in 20 blocks | Rebuilt image byte-identical (SHA-256), size MATCH; carve CONTIGUOUS from 5.00 s before `0x34` to 1.99 s after `0x37` |

| With the details/health additions: 60,000 eight-byte at ~2,700 frames/s, then 20,000 mixed | 80,000 captured, 0 overruns; unique IDs 1,031 (the sender's 1,024 standard + 7 extended); PSRAM ring peak 0.006 %, staging peak 8 of 2,048 |
| Synthetic flash, 500 KB image in 500 blocks, ~2,900 frames/s / 64 % load during transfer | HA showed live progress (67 % mid-way); final 512,000/512,000 B, 500 blocks, 100 %; 0 overruns over 154,557 frames; rebuilt image byte-identical; carve CONTIGUOUS, one `0x34`, ends 2.00 s after `0x37` |

The flash test found two carving bugs, now fixed: the `0x34` trigger frame was
written twice, and the pre-roll was limited only by frame count, so on a quiet
bus it reached back ~30 minutes.

Not yet tested: above ~65 % bus load (the CANable sends about 2,500-2,700
frames/s at most; more than 32 frames in flight makes it drop its own read-back
copies), the CANable logging someone else's traffic at high load, and a real
ECU flash.

## The CANable as reference logger

`scripts/canable_log.py` writes the same candump format as `canbin2candump.py`.

- **gs_usb (Candlelight):** opened listen-only (bus monitoring), never ACKs;
  hardware timestamps when the firmware fills them (falls back to PC time when
  it sends 0). It refuses firmware that doesn't offer listen-only.
- **Factory slcan firmware:** can't listen silently (its `M1` is undone by `O`),
  so it ACKs like an ECU. The script refuses it unless you pass `--allow-ack`.
- python `gs_usb` only has bit timings for 48 and 80 MHz CAN clocks; for others
  (Elmue's firmware runs at 160 MHz) the script derives the timing itself,
  picking the sample point closest to 87.5 %.
