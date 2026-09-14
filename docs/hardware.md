# Hardware map

This document describes the pin assignments used by `apps/controller`. Update it
whenever the wiring or board revision changes.

## Direct GPIO buttons

| Input | ESP32 GPIO |
| --- | ---: |
| A | 19 |
| B | 18 |
| X | 17 |
| Y | 23 |
| D-pad up | 25 |
| D-pad left | 27 |
| D-pad down | 26 |
| D-pad right | 14 |

These inputs are active low and use the ESP32 internal pull-ups.

## Analog sticks

| Axis | ESP32 GPIO / ADC1 channel |
| --- | --- |
| Left X | GPIO32 / channel 4 |
| Left Y | GPIO33 / channel 5 |
| Right X | GPIO34 / channel 6 |
| Right Y | GPIO35 / channel 7 |

Calibration values are currently board-specific and live in `analog_sticks.c`.

## Other input devices

| Device | Connection | Purpose |
| --- | --- | --- |
| TCA9555 at `0x20` | I2C0: SDA 21, SCL 22, 100 kHz | L/R, stick clicks, Select, Start, Capture, Home |
| ESP32 touch pads | Touch 0 (ZR), Touch 4 (ZL) | Capacitive digital triggers |
| CH9350L | UART2 RX on GPIO16, 115200 baud | Optional mouse input |

The abandoned CST816 touchscreen prototype is retained under
`experiments/touchscreen-Nintendo-switch`; it is separate from the capacitive
ZL/ZR inputs used by the active firmware.

## Programming controls

The board requires manual BOOT/download and EN/RESET button operation for flashing.
See `build-and-flash.md` for the sequence.
