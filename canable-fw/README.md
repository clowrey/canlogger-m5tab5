# CANable 2.0 firmware for listen-only logging

The reference logger (`../scripts/canable_log.py`) and the desk test senders
need a CANable that can listen without ACKing and can transmit in external
loopback. The CANable 2.0's factory firmware can't do the first, so it was
reflashed. These notes are for a **DYKB "CANable v2.0 nano"** (AliExpress,
STM32G431), but apply to official CANable 2.0 boards too.

Used: **Elmue's CANable 2.5 Candlelight** (gs_usb) built with the `DykbNano`
board from this folder. It shows up as USB 1d50:606f "Candlelight 2.5 - DYKB
CANable 2.0 nano", with listen-only, loopback and µs hardware timestamps (CAN
clock 160 MHz). Windows binds WinUSB to it by itself.

| File | What |
|---|---|
| `elmue-dykbnano.patch` | Adds the `DykbNano` board to Elmue/CANable-2.5-firmware-Slcan-and-Candlelight @ 12aa9fa (firmware 0x261001). |
| `Make_G431_Candle_DykbNano` | Makefile for that board (goes in the firmware repo's `Build/`). |

No firmware binaries are kept here: Elmue's repository has no license file, so
build it yourself (below).

## Firmware compared

| Firmware | Listen-only | Timestamps | Measured |
|---|---|---|---|
| Factory slcan (normaldotcom canable2 16e7497) | No: `O` resets `M1` to normal mode, so it ACKs | PC only | not used |
| normaldotcom budgetcan_fw `board_canablev2` | Yes | Reports them, always sends 0 | ~500 frames/s sending in loopback. Its main loop moves one frame each way per 1 ms tick, so as a logger it tops out near 1,000 frames/s and drops silently above that. |
| **Elmue Candlelight 2.5** | Yes (`FDCAN_MODE_BUS_MONITORING`) | Hardware, µs | 2,555-2,766 frames/s sending in loopback, all 8-byte (~57-61 % of 500 kbit/s) |

Upstream candleLight_fw has no STM32G431 target, and canable.io's web updater
had an expired TLS certificate (2026-10-28).

## Pins (DYKB board)

Read from the disassembled factory firmware: CAN PB8/PB9 (AF9, FDCAN1),
transceiver standby PA0 driven low, transceiver IO power PC13 driven high, LEDs
PA15/PB11. Elmue's `OpenlightLabs` board has the same CAN and LED pins but
leaves PA0 and PC13 alone, so the patch adds a `DykbNano` board that drives
them (`CAN_TRX_ENABLE_PIN` for PA0, a new `CAN_VIO_ENABLE_PIN` for PC13).

## Building (Linux / WSL, arm-none-eabi-gcc + make)

```bash
git clone https://github.com/Elmue/CANable-2.5-firmware-Slcan-and-Candlelight.git elmue_canable
cd elmue_canable && git checkout 12aa9fa
git apply <this repo>/canable-fw/elmue-dykbnano.patch
cp <this repo>/canable-fw/Make_G431_Candle_DykbNano Build/
make -s -f Build/Make_G431_Candle_DykbNano
```

Output: `Build/STM32G431_Candlelight_DykbNano/Candlelight2.5_DykbNano_STM32G431_0x261001.bin`.

## Flashing (STM32CubeProgrammer)

1. Back up the factory firmware first. Hold BOOT while plugging in: it appears as
   "DFU in FS Mode" (0483:df11). Then:
   ```bash
   STM32_Programmer_CLI -c port=usb1 -u 0x08000000 0x20000 backup_factory.bin
   ```
2. Flash:
   ```bash
   STM32_Programmer_CLI -c port=usb1 -e all -d Candlelight2.5_DykbNano_STM32G431_0x261001.bin 0x08000000 -v
   ```
3. Unplug and plug in again **without** BOOT. (Starting the firmware straight from
   DFU with `-s` left USB dead.)

Elmue's firmware can enter DFU by command (see its manual), so later updates
don't need the BOOT button. `dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D <file>.bin`
works too. To go back, flash the backup the same way.
