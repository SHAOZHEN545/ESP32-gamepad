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

The CH9350L's two USB host connections are labeled port 1 (`DP/DM`) and port 2
(`HP/HM`) in the chip documentation. Either can accept a mouse when both are
wired on the module; the physical upper/lower position depends on the board.

The abandoned CST816 touchscreen prototype is retained under
`experiments/touchscreen-Nintendo-switch`; it is separate from the capacitive
ZL/ZR inputs used by the active firmware.

### CH9350 aim mapping

In Switch mode, holding the ZR touch pedal or the mouse right button selects
the CH9350 aim path when mouse input has not been disabled at startup. Either
input also reports ZR pressed, so the right mouse button can draw and release
a bow in Zelda games.
The mouse left button has no mapping. In Switch mode, horizontal mouse motion
is mapped with inverted sign to synthetic gyro Z (yaw), and vertical mouse
motion to gyro X (pitch). Both signs were flipped after the first hardware
test reported reversed aim on both axes. The
Switch's `0x40` IMU-enable command must be active before those samples are sent.
The CH9350 does not provide acceleration, so all accelerometer axes are zero.
Mouse deltas are drained at the roughly 10 ms Switch report cadence, scaled
and clamped to the signed 16-bit report range, and reset to zero whenever ZR
is not held. `BOARD_MOUSE_GYRO_YAW_RAW_PER_DELTA` controls horizontal aim
(mouse X, currently 72); `BOARD_MOUSE_GYRO_PITCH_RAW_PER_DELTA` controls
vertical aim (mouse Y, currently 96). Smaller values slow the corresponding
axis; each change requires a rebuild and flash. Both axes previously used 80,
and the original test value was 160.

In XInput mode, the original ZR touch pedal still enables mouse-to-right-stick
aim and sets the right trigger. The mouse buttons are not mapped in that mode.
Holding A during startup disables all CH9350 input in either mode.

## Programming controls

The board requires manual BOOT/download and EN/RESET button operation for flashing.
See `build-and-flash.md` for the sequence.
