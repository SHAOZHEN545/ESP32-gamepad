# Repository guidance

## Active firmware

- Treat `apps/controller` as the only actively maintained controller firmware.
- Treat `examples/basic-wired-gamepad` as upstream reference code.
- Treat everything under `experiments` as historical reference unless the user
  explicitly asks to revive an experiment.
- Do not incorporate the `Gyro-mouse` branch during repository-only refactoring.

## Compatibility and validation

- Preserve compatibility with ESP-IDF 5.0.1 and the ESP32 target.
- Build with `scripts/build-controller.ps1 -Clean` from an ESP-IDF 5.0 PowerShell.
- Commit `948c4c2` is the hardware-verified pre-refactoring baseline.
- Keep structure-only changes separate from controller behavior changes.
- A successful build is not sufficient for input changes. Record physical Switch
  and XInput results in `docs/validation.md` after the user tests the hardware.
- Flashing requires the user to operate the board's BOOT/download and EN/RESET
  buttons. Do not report a flash as complete without that physical step.

## Source ownership

- Board pins and device constants belong in `apps/controller/main/board_config.h`.
- Device drivers should not contain HOJA report mapping.
- HOJA report mapping belongs in `controller_input.c`.
- Startup mode selection and application startup belong in `app_main.c`.
- Update `docs/hardware.md` when wiring or calibration changes.
- Update `docs/project-history.md` when moving or retiring firmware variants.

The root of this repository is also an ESP-IDF component. Changes under `cores`,
`include`, `utilities`, or other root library directories affect every app and
experiment in the current checkout. Document the reason for shared-library changes
and validate both Switch and XInput modes.
