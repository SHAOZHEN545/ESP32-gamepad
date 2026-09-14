# ESP32 multi-mode controller

This repository contains a custom ESP32 controller built on top of
[HOJA-LIB-ESP32](https://github.com/HandHeldLegend/HOJA-LIB-ESP32). The current
firmware supports Nintendo Switch Pro Controller and Bluetooth XInput modes,
physical buttons, two analog sticks, capacitive ZL/ZR triggers, a TCA9555 button
expander, and optional CH9350 mouse input.

The hardware-verified firmware is in [`apps/controller`](apps/controller). It was
last verified from commit `948c4c2` with ESP-IDF 5.0.1 before the repository
reorganization.

## Repository map

- `apps/controller`: the controller firmware under active development.
- `examples/basic-wired-gamepad`: the original upstream example.
- `experiments`: retained prototypes and intermediate implementations.
- `docs`: build, hardware, validation, and project-history notes.
- `cores`, `descriptors`, `include`, `resources`, `utilities`: the shared HOJA
  component and this fork's protocol fixes.

## Start here

1. Read [`docs/build-and-flash.md`](docs/build-and-flash.md).
2. Build from `apps/controller` with ESP-IDF 5.0.1.
3. Flash while manually putting the board into download mode.
4. Follow [`docs/validation.md`](docs/validation.md) to test Switch and XInput.

The `Gyro-mouse` branch is intentionally outside the current refactoring scope.
