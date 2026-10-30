# Plan: a second, CAN FD channel

Status (2026-10-29): **option B is built** (`fd_channel: driver: gs_usb`; see
the README's "Second channel" section and docs/TESTING.md section 5). Option A
is not started.

Goal: log a CAN FD bus alongside the existing classic channel, both at full
load, listen-only, with every lost frame counted, into the same capture files
on one timeline. The Tab5's own TWAI controllers are classic CAN only, so the FD
channel needs an external controller. Two ways in:

- **A. MCP2518FD on SPI** via the M5-Bus header on the back, read by Jonny's
  `mcp2518fd_lite` driver from Battery-Emulator.
- **B. STM32G4 on USB**: a CANable 2.0 (STM32G431, FDCAN) plugged into the
  Tab5's USB-A host port, running Elmue's Candlelight firmware.

Either one is a **build option**: without it in the YAML, the firmware is built
exactly as today.

## What the FD bus can throw at us

At 100 % load, 500 kbit/s arbitration:

| Frames | 2 Mbit/s data | 5 Mbit/s data |
|---|---|---|
| 64 bytes | ~2,700 frames/s, ~175 KB/s | ~5,400 frames/s, ~350 KB/s |
| 0-8 bytes | ~8,600 frames/s | ~9,500 frames/s |

The classic channel adds up to ~4,000 frames/s. Both together are well under
1 MB/s to the SD card and leave the 16 MB PSRAM ring with ~30 s of headroom.
The hard part is the **controller's own buffer**: how long the Tab5 may be busy
before the FD controller has to drop a frame.

## Shared work (needed by both options)

1. **File format `CANBIN02`**: a channel number in every record, and FD records
   (FDF/BRS/ESI flags, up to 64 data bytes). Use variable-length records (16-byte
   header + payload rounded up to 8) so classic frames stay small. Bump the
   header version; the scripts keep reading `CANBIN01`.
2. **Scripts**: `canbin2candump.py` (candump FD syntax `id##<flags><data>`),
   `compare_logs.py`, `reassemble_uds.py` (ISO-TP over FD: 64-byte CAN_DL, escape
   sequence for first frames > 4095 bytes), `tab5_fetch.py` unchanged.
3. **One timeline**: the FD controller stamps frames with its own 1 µs counter.
   Read that counter against `esp_timer` about once a second and map each stamp,
   so the two channels line up to a few µs. Handle the 32-bit wrap (~71 min).
4. **Second staging ring** in internal RAM (variable-length, ~64 KB: ~800
   64-byte frames or ~2,700 short ones) feeding the same capture task and PSRAM
   ring. Overflows anywhere on the FD path add to a per-channel overrun count and
   set `GAP` on the next record, exactly as on channel 0.
5. **Entities and screen**: per-channel frames, frames/s, bus load (FD-aware bit
   count), overruns, bus state; the details page gets an FD section.
6. **Build option** in `components/can_logger/__init__.py`: an optional
   `fd_channel:` block. Present → `cg.add_define("USE_CAN_LOGGER_FD")` plus the
   option's own define, and the IDF pieces it needs. Absent → nothing extra
   compiles (all FD sources wrapped in `#ifdef`).

   ```yaml
   can_logger:
     id: canlog
     rx_pin: GPIO54
     bitrate: 500000
     fd_channel:                 # omit for a classic-only build
       driver: mcp2518fd         # option A, or: gs_usb (option B)
       bitrate: 500000
       data_bitrate: 2000000
       # option A only:
       sck_pin: GPIO5
       mosi_pin: GPIO18
       miso_pin: GPIO19
       cs_pin: GPIO16
       int_pin: GPIO45
   ```

7. **FD test traffic**: extend `canable_testgen.py` / `canable_uds_sim.py` to
   send FD frames (Elmue's firmware does FD; the python `gs_usb` library would
   need FD frame support or Elmue's own protocol). Same loopback trick as now.

## Option A: MCP2518FD on SPI

### The driver to start from

`Software/src/lib/mcp2518fd_lite/` in
[dalathegreat/Battery-Emulator](https://github.com/dalathegreat/Battery-Emulator),
by Jonny (jonny5532): added 2026-10-08 in "CAN: Add replacement interrupt-driven
MCP2515/MCP2518FD drivers" (29cc73db, replacing ACAN2517), last changed
2026-10-21 ("Reduce FD TX buffer size, tidy defs", bf745f86). GPL-3.0, same as
this repo; keep its notice. It runs on `lib/mcp2515_lite/async_spi_bus.*`.

Why it fits:

- **All SPI work happens in an IRAM interrupt.** Each step is one SPI
  transaction, started from the SPI-done ISR (level 3, `ESP_INTR_FLAG_IRAM`) and
  the INT-pin ISR; it touches only internal RAM. So it keeps reading frames while
  the flash cache is off (OTA, settings saves), the same property our TWAI path
  has. A task-based reader would stall there.
- Per frame: FIFO status (10 bytes), header + first 8 bytes (18), the rest of a
  long payload (up to 58), UINC (3). At its 16 MHz SPI clock (0.4 × SYSCLK, 40 MHz
  crystal) that's ~60 µs per 64-byte frame and ~30 µs per short one, against
  365 µs and 116 µs between frames on a full bus: 17-35 % of the bus time.
- Variable-length host ring, RX overflow (`RXOVIF`) detection, missed-INT poll,
  oscillator autodetect, bit-timing calculation, debug dump.

What we change:

| Change | Why |
|---|---|
| Listen-only mode (`REQOP` 3) instead of Normal FD (0) | it must never ACK |
| Drop the TX FIFO and TX path; RX FIFO gets all 2 KB: 26 objects at 64 bytes + timestamp | depth: 26 × 116 µs ≈ 3 ms minimum slack at the worst frame rate, ample for an ISR |
| Enable the time base (`CiTSCON` TBCEN, prescaler for 1 µs) and `RXTSEN` on the RX FIFO; read the 4-byte stamp | one timeline (shared item 3) |
| Push straight into our FD staging ring; count `RXOVIF` and ring-full as overruns | integrity gate per channel |
| Replace Arduino calls (`pinMode`, `delay`, `micros`, `millis`, `DEBUG_PRINTF`) with ESP-IDF | ESPHome/IDF build |
| Port `async_spi_bus` to the P4: GPIO interrupt (it uses the ESP32/S3 `GPIO_*_NMI_INT` status registers and `ETS_GPIO_NMI_SOURCE`), SPI clock source (it assumes the 80 MHz APB clock), check `spi_ll` calls against the P4 HAL | different chip |
| Keep its host tests (`test/can_spi_driver_tests.cpp`) for the util/ring code | regression |

Open question: Linux's `mcp251xfd` driver uses CRC-checked SPI reads and RAM ECC
on these chips because of SPI read errata. Jonny's driver doesn't. Decide after
bench testing (a corrupted header shows as a bad DLC/ID burst).

### Hardware

- Any MCP2518FD board with a **40 MHz crystal** (20 MHz halves the SPI clock and
  limits the data rate) and an **FD-rated transceiver** (MCP2562FD, TCAN1042 or
  similar): MikroE MCP2518FD Click, Waveshare, Seeed.
- M5-Bus on the Tab5's back: MOSI G18, MISO G19, SCK G5, CS and INT on two of the
  plain GPIOs (G16, G45, G2, G3, G4, G47, G48, G35, G51), 3V3 and 5V. Check the
  schematic that the chosen ones are free. SPI2 is unused (SD is on SDMMC, the C6
  on SDIO).
- **Silent in hardware**, like channel 0: don't connect the MCP2518FD's TXCAN to
  the transceiver; pull the transceiver's TXD up (recessive). Then a firmware bug
  can't drive the bus.

### Steps

1. Wire a board; port `async_spi_bus` + driver; read registers and the RAM
   test pattern.
2. Listen-only + timestamps + our ring; FD test traffic from the CANable in
   loopback.
3. Shared items 1-6; then load tests: 64-byte at 2 and 5 Mbit/s, short frames
   back to back, both channels together, an OTA during capture.

## Option B: STM32G4 on the USB-A port

### What exists

Elmue's Candlelight firmware (already on the CANable, with our board patch in
`canable-fw/`) has:

- CAN FD (`GS_ReqGetCapabilitiesFD`, data bit timing), listen-only (bus
  monitoring), 1 µs hardware timestamps on every frame (wrap after ~71 min), and
  `GS_ReqGetTimestamp` to read the counter (shared item 3).
- A blob mode (`ELM_DevFlagSendUsbBlobs`) that packs several frames into one USB
  transfer (3 packets), fewer USB transactions per frame.
- Buffers: `CAN_QUEUE_SIZE` 64 (to the bus), `HOST_QUEUE_SIZE` **70 frames** (to
  the host). A full host queue raises `APP_UsbInOverflow`: loss is reported, not
  silent.

### What we write

1. Turn on the USB-A port's 5 V (`pi4ioe2` P3, "USB Power" in ESPHome's Tab5
   config; our YAML doesn't drive it yet).
2. A gs_usb host client in the component on the ESP-IDF USB Host Library:
   install the host stack, register a client, on attach match the CANable's
   VID:PID (`1d50:606f`), claim the interface, send Elmue's control requests
   (capabilities, bit timing, data bit timing, mode = listen-only + FD +
   timestamps + blobs), then keep ~16-32 bulk IN transfers queued on the IN
   endpoint. Each completion parses frames into the FD staging ring and
   resubmits. Error/overflow frames → overrun count.
3. Hot-plug: on detach, mark the channel down; on reattach, set it up again and
   flag `GAP`.
4. ESPHome strips IDF components it thinks unused: re-include `usb` (as the
   component already does for `esp_driver_twai`, `fatfs`).

### The weak point and its fix

The USB host stack runs in tasks, not IRAM interrupts, so it stops while the
flash cache is off. Then only the CANable's 70-frame host queue absorbs the bus:
~8 ms at 8,600 frames/s, ~26 ms at 2,700. A flash erase can take longer. Fixes,
cheapest first:

- Raise `HOST_QUEUE_SIZE` in our patched Elmue build. The G431 has 32 KB SRAM;
  ~300 frames (~24 KB) gives ~35 ms at the worst rate. Measure free RAM first.
- Keep flash writes out of capture: no OTA while frames are arriving, settings
  saved at Safe shutdown rather than on change.

### Testing needs a second FD sender

The CANable becomes the device under test, so desk tests need another FD-capable
adapter as the sender (a second CANable 2.0 is the cheap way). Also check that the
DYKB board's transceiver is FD-rated (read its marking).

## Comparison

| | A: MCP2518FD on SPI | B: CANable on USB-A |
|---|---|---|
| Hardware | buy a board, wire 7 pins to the M5-Bus | plug in the CANable you have (or a second one) |
| Silent on the bus | in hardware (TXCAN not connected) | firmware only (bus-monitoring mode), unless the board's TXD is modded |
| Survives flash writes (OTA, saves) | yes: the reader is an IRAM ISR, like channel 0 | only for as long as the CANable's queue lasts (8-26 ms now, ~35 ms with a bigger queue) |
| Controller buffer | 26 frames (~3-9 ms), drained by an ISR within µs | 70 frames now, ~300 possible |
| Max rate | ~17-35 % of SPI time at full load: fine at 2 and 5 Mbit/s | USB FS: ~5,400 × 2 packets/s or 8,600 × 1, fine; blobs help |
| Timestamps | chip's 1 µs counter, mapped to `esp_timer` | firmware's 1 µs counter, mapped via `GS_ReqGetTimestamp` |
| Code we write | port ~1,000 lines (driver + SPI bus) to the P4 and IDF, plus listen-only/timestamp changes | new gs_usb host client (~500-800 lines), USB 5 V, hot-plug; small firmware tweak on the G4 |
| Other firmware to maintain | none | our Elmue patch (already exists) |
| Desk testing | existing CANable as the FD sender | needs a second FD adapter as the sender |
| Failure modes | wiring, SPI errata, P4 interrupt port | enumeration, cable, hot-plug, USB stack stalls |
| CPU | ~10 % of one core at the worst frame rate (ISRs) | similar, in a high-priority task |

## Recommendation

Both fit behind the same `fd_channel:` option and share most of the work (file
format, scripts, timeline, staging ring, entities), so build the shared part
first.

- If the FD channel must be as trustworthy as channel 0, **A**. Its reader keeps
  running through flash writes, and it can be made physically unable to transmit.
- If plug-and-play matters more, **B**, with the larger host queue in our
  firmware patch and no flash writes during capture. Every loss is still counted.

Suggested order: shared items 1-4, then a one-day **B spike** (enumerate the
CANable from the Tab5, stream frames, measure how many it drops during an OTA and
a settings save at full load). That number decides whether B is enough or A is
worth the wiring.
