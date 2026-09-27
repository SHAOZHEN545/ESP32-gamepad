# Project history

The repository began as a fork of HOJA-LIB-ESP32. Controller development then
progressed by copying and extending the upstream example. The current layout keeps
that history while giving ongoing work a single home.

| Directory | Status | Purpose |
| --- | --- | --- |
| `apps/controller` | Active | Combined Switch/XInput controller; successor to `Combo-NS-XBox` |
| `examples/basic-wired-gamepad` | Upstream reference | Original buildable HOJA example |
| `experiments/minimal-Nintendo-switch` | Retained experiment | Minimal Switch input and troubleshooting firmware |
| `experiments/mouse-Nintendo-switch` | Retained experiment | Early CH9350 mouse-to-stick work |
| `experiments/Nintendo-switch-controller` | Retained experiment | Switch-only integrated controller stage |
| `experiments/XBox-controller` | Retained experiment | XInput-only integrated controller stage |
| `experiments/touchscreen-Nintendo-switch` | Abandoned experiment | CST816 touchscreen mapped as an analog input |

The experiment directories are snapshots for reference. They still build against
the shared library in the current checkout, so Git commits remain the authoritative
record of their original behavior.

## Shared-library changes in this fork

- XInput reports include left- and right-stick click buttons.
- Switch report handling was adjusted to preserve held-button state.
- The HOJA button task stack was increased from 2048 to 4096 bytes.
- Switch standard reports now include optional three-sample IMU payloads after
  the Switch enables IMU. This shared transport exists so the active controller
  can map CH9350 mouse deltas to native Switch gyro aiming; it is gated to the
  Switch core and does not alter XInput report handling.

These changes are part of the working controller baseline and must be reviewed when
merging future upstream HOJA updates.
