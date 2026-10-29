# Testing

Sections 1-3 are desk tests with the Tab5 and a CANable 2.0 wired to the same
two CAN wires (CAN-H, CAN-L, plus termination), nothing else on the bus.
Section 4 checks the Tab5 on a live bus, with the CANable only listening.

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
python tools/tab5_fetch.py <tab5-ip> seg -o captures
python scripts/reassemble_uds.py captures/segNNNN.canbin --tx-id 0x602 -o uds_rebuilt.bin
python scripts/canbin2candump.py captures/segNNNN.canbin -o tab5.log -i can0
python tools/compare_logs.py subset tab5.log uds_sent.log
```

`canable_uds_sim.py` plays both sides with background traffic throughout:
`0x10 02`, `0x27` seed/key, a multi-frame `0x34` RequestDownload, `0x36`
TransferData blocks with ISO-TP flow control from the ECU id (`0x612`), `0x37`,
then more traffic. It saves the image it sent as `<prefix>_image.bin`. Pass
when:

- `reassemble_uds.py` says MATCH and the rebuilt image's SHA-256 equals
  `uds_image.bin`'s;
- `compare_logs.py subset` says SUBSET with no frames only in the capture.

## 3. Screen, details and rate under load

Run section 1's `--full` sender with `states --watch` and look at the screen:
frames/s and bus load update once a second, overruns stay 0. Tap DETAILS
(bottom left) for the details page. `tools/tab5_api.py states <tab5-ip> --all` prints every
entity (unique IDs, busiest IDs, buffers, SD, health).

Only one program can hold the CANable at a time; a second one stops with
"CANable is busy".

## 4. Live bus

Connect both to the bus; both only listen. Log with the CANable for as long as
the Tab5 should be checked, then compare:

```bash
python scripts/canable_log.py -o canable.log            # Ctrl+C to stop
python scripts/canbin2candump.py segNNNN.canbin -o tab5.log -i can0
python tools/compare_logs.py subset tab5.log canable.log
```

Use the segment(s) covering the CANable's run (convert each and concatenate the
logs if it spans a rotation). Pass when SUBSET reports 0 reference frames
missing and the Tab5 shows 0 overruns. Frames only in the Tab5 capture are
fine as long as the CANable reported overflows for them (see below). If the bus
carried a firmware download, `reassemble_uds.py` on the segment should say
MATCH.

## Results

Desk (2026-10-28, ESPHome 2026.7.2):

| Test | Result |
|---|---|
| 20,200 mixed frames (2,020 extended, 208 RTR, lengths 0-8), ~500 frames/s | Captured, 0 overruns; `exact`: identical order and content |
| 80,000 frames incl. 40,000 eight-byte at 2,441 frames/s (~56 % bus) | All counted, 0 overruns, 0 bus errors |
| 60,000 eight-byte at 2,555-2,766 frames/s (57-61 % bus), screen redrawing | All counted, 0 overruns; rate and load shown live |
| 60,000 eight-byte at ~2,700 frames/s, then 20,000 mixed | 80,000 captured, 0 overruns; unique IDs 1,031 (the sender's 1,024 standard + 7 extended); PSRAM ring peak 0.006 %, staging peak 8 of 2,048 |
| Synthetic flash, 20 KB image in 20 blocks | Rebuilt image byte-identical (SHA-256), size MATCH |
| Synthetic flash, 500 KB image in 500 blocks, ~2,900 frames/s / 64 % load during transfer | 0 overruns over 154,557 frames; rebuilt image byte-identical, size MATCH |

(The synthetic flashes ran on earlier firmware that also carved each session
into its own file; the rebuilds were from that file.)

Live vehicle bus (2026-10-28): ~280 ids, ~2,300 frames/s, ~47 % load before
stuff bits, frames down to 95 µs apart. Tab5 and CANable side by side, checked
with `compare_logs.py subset`:

| Test | Result |
|---|---|
| 60 s, CANable with the old one-read-at-a-time loop | All 136,975 CANable frames in the Tab5 capture, same order; the Tab5 had 196 more, in 27 bursts the CANable dropped. Tab5: 0 overruns, largest gap between frames 3.3 ms |
| 120 s, CANable with 256 queued USB transfers | 273,389 frames, identical frame for frame; CANable overflows 0, Tab5 overruns 0 |
| 22 min across an ECU firmware update over UDS (diagnostic traffic to a dozen ECUs, one ~43 KB download), spanning a segment rotation | All 2,740,311 CANable frames in the Tab5 capture, same order; the Tab5 had 144 more, in the CANable's one overflow (a ~70 ms gap). Tab5: 0 overruns. The download rebuilt from the Tab5 segment and from the CANable log (repacked as `.canbin`): identical images (SHA-256), size MATCH |

Not yet tested: above ~65 % bus load (the CANable sends about 2,500-2,700
frames/s at most; more than 32 frames in flight makes it drop its own read-back
copies).

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
- python `gs_usb` reads one frame per USB transfer, ~0.3 ms per round trip on
  Windows, and the CANable holds only a few frames meanwhile: on a busy bus it
  overflowed and dropped bursts of 30-40 frames. The script keeps 256 bulk
  transfers queued instead (libusb async on pyusb's handle). The CANable reports
  its own drops as RX-overflow error frames, shown as "CANable overflows": the
  log is a complete reference only while that stays 0. Over a 22-minute run it
  still reported one (a ~70 ms pause on the PC), so for long runs the Tab5's
  capture is the complete record and the CANable log the cross-check.
- A frame left queued from the previous run (minutes-old timestamp) is drained
  before logging starts.
