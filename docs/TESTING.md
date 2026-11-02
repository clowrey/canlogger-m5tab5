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

## 5. The CAN FD channel, on the desk

With the CANable in the Tab5's USB-A port and nothing on its CAN wires, a test
build sends frames through the CANable's **internal loopback**. Its FDCAN never
drives the bus in that mode. Add this to `fd_channel:` and flash:

```yaml
    test_loopback_frames: 200000
```

After the CANable starts, the Tab5 sends that many numbered frames on two ids:
`0x1FD` (standard) and `0x1FD1FD0` (extended). They cycle through every FD
length (4-64 bytes), with BRS, and every 17th is a classic 8-byte frame. At
most 32 are in flight. The log then says:

```
FD loopback test: sent 200000, received 200000, sequence errors 0, 6466 frames/s
```

That checks the USB path. For the SD path, fetch the segment and check that the
bus-1 frames hold every number once, in order:

```bash
python scripts/canbin2candump.py seg0046.canbin --bus 1 -o fd.log
```

Remove `test_loopback_frames` again for real use: it puts the CANable in
loopback instead of listen-only.

## 6. Real CAN FD on the wires

This needs a second CANable with Elmue's firmware as the sender, on a PC and on
the same two wires as the FD channel. The rates must match `fd_channel:`.

```bash
python scripts/canable_fd_testgen.py -n 300000 --fd-only -p fd1                # every length, 0-64 bytes
python scripts/canable_fd_testgen.py -n 150000 --len 64 --start 1000000 -p fd2 # 64-byte frames
python scripts/canable_fd_testgen.py -n 34000 --start 2000000 -p fd3           # every 17th frame classic, some RTR
python tools/tab5_fetch.py <tab5-ip> seg -o captures
python scripts/canbin2candump.py captures/segNNNN.canbin -o tab5.log
python tools/compare_logs.py --iface can1 exact tab5.log fd1_sent.log fd2_sent.log fd3_sent.log
```

Pass when `exact` says IDENTICAL and FD overruns stay 0. Use `-d 5000000` for a
5 Mbit/s build. `--no-brs` keeps FD frames at the nominal rate, `--classic` sends
only classic frames, and `--gap-ms` paces them.

In external loopback the sender reads its own frames back inside the chip, not
from the wires, so its read-back says nothing about what's on the bus. Only the
Tab5's capture does.

If the sender says "no TX echo" while frames are still arriving at the Tab5, it
is in the bad state described under the results. Restart it: send it DFU detach
(class request 0 to interface 1, which Windows lets you do on this firmware), then
run `STM32_Programmer_CLI -c port=usb1 -g 0x08000000`.

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

CAN FD channel, desk loopback (2026-10-29, CANable 2.0 nano with Elmue's
firmware on USB-A, 500 kbit/s / 2 Mbit/s):

| Test | Result |
|---|---|
| 200,000 numbered frames through internal loopback, 4-48 bytes (the first test build's pattern skipped 64) plus classic 8-byte, std and ext ids | Sent 200,000, received 200,000, 0 sequence errors, ~6,466 frames/s at the USB side. In the segment (`CANBIN02`, fetched with resume): all 200,000 on bus 1, in order, payloads, flags and ids correct, 0 GAP flags, 6,652 frames/s by timestamp. FD overruns 0, FD staging peak 24 of 2,048 |

Both channels on real wires (2026-10-31): the FD channel's CANable and Port A on
one bus, with a second CANable (MKS Makerbase, budgetcan firmware) as sender. Its
firmware got stuck resending one queued 0-byte classic frame, holding the bus at
100 %, about 7,714 frames/s. These are classic frames, because that firmware
can't send FD:

| Test | Result |
|---|---|
| 100 s at full load | 771,913 frames on each channel (788,207 in the segment, with an earlier 2 s burst), identical; 0 overruns, 0 GAP flags; buffer peak 2.5 %, staging peaks 26 and 28 of 2,048 |
| 150 s at full load, screen brightness changed and held across two preference saves, one segment rotation | 1,157,864 more frames, totals identical on both channels; 0 overruns |
| 110 s at full load while downloading a 38 MB file over Wi-Fi | 849,131 frames, totals identical; 0 overruns. Buffer peak 26.8 % (Wi-Fi and SD share one SDMMC host, so the download slows SD writes); the web server answered nothing else meanwhile; download not finished after ~90 s |
| Timestamps, bus 1 against bus 0 | Median 69 µs, always within about ±1 ms over 197 s including a 95 s idle gap. The first firmware drifted 950 µs/s (this CANable's clock is 0.095 % slow) until rate tracking was added |
| Tab5 restart on the flooded bus | Files closed in 30 ms before the reboot. The FD channel starts ~1.5 s after Port A, then loses a few frames at start (8 fewer than Port A; a 2.7 ms hole), reported as 2 FD overruns with GAP flags; then second-by-second match |
| OTA on the flooded bus | OTA completed and logging resumed. **The file ends about when the OTA starts**: the SD writer stops and was still stuck 6 s after the OTA finished ("files NOT closed"), so frames from the OTA's start until the reboot are lost, on both channels. Don't OTA during a capture |

Real CAN FD on the wires (2026-11-02): an MKS Makerbase CANable (Elmue's
Multiboard firmware) sending on a PC, with the FD channel's DYKB CANable and Port A
listening on one desk bus, nominal 500 kbit/s. All with BRS unless stated; every
result compared frame by frame with `exact` unless stated:

| Test | Result |
|---|---|
| 2 Mbit/s: 300,000 frames of every length 0-64 bytes at 6,450 frames/s, then 150,000 of 64 bytes at 2,789 frames/s | 450,000 identical, 0 FD overruns, 0 GAP flags; buffer peak 1.8 %, staging peak 29 of 2,048 |
| 5 Mbit/s: 800,000 of every length at ~9,300 frames/s, then 200,000 of 64 bytes at 5,274 frames/s | 1,000,000 identical, 0 FD overruns, 0 GAP flags; buffer peak 16.5 %, staging peak 41 |
| 2 Mbit/s: 34,000 with every 17th classic (2,000 classic, 182 of them RTR) at 5,485 frames/s | 34,000 identical on the FD channel |
| An earlier 5 Mbit/s run at ~9,300 frames/s | **One real loss:** 133 frames missing during a 13.5 ms silence, 20 s in; staging peak 93. Reported as 1 FD overrun and a GAP flag, but the flag landed 86 ms and 836 frames after the hole, since the CANable reports errors late. Cause of the pause not found; it didn't recur in the 1,000,000-frame run. This run also hit the sender fault in the next row, so it isn't in the identical totals above |
| Sender faults, three times (two after starting the firmware, one mid-run after running well) | The Makerbase went bad: both Tab5 channels missed the same frames (59 of 458 classic; about 70 % of FD frames of 12-64 bytes at the nominal rate, nearly always the same ones on a repeat), then its USB read-back stopped. After restarting it, those frames all arrived. Frames that fail on the wire are **not** counted as FD overruns, because no valid frame reaches the CANable. Cause not found |
| 960 fixed-pattern FD frames (0x00, 0xFF, 0x55, 0xAA, 0x0F, 0x01; 8-64 bytes; nominal rate and BRS) | All arrived |

Port A on the same bus: it can't decode FD frames, and being listen-only it never
disturbs the bus. With frames ~3 ms apart it logged all 100 classic frames among
1,600 FD frames. Back to back it logged only 396 of 2,000 classic frames, all of
them real and in order, with no overruns counted: a classic controller can't
follow a bus like that. Over 1.5 million FD frames at 5 Mbit/s it logged 1 false
classic frame (`13E#FD71DA277B`). So on a bus carrying FD, use the FD channel's
record.

Not yet tested on the FD channel: a live FD bus, and data rates above 5 Mbit/s.

Not yet tested on Port A: 8-byte frames above ~65 % bus load (with 0-byte frames
it held 100 %, above).

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
