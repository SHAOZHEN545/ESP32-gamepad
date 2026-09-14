# Validation log

## Verified baseline — 2026-09-13

- Commit: `948c4c2`
- ESP-IDF: 5.0.1
- Result: clean build, flash, and boot passed.
- Nintendo Switch: initial connection required **Controllers → Change Grip/Order**;
  pairing then succeeded and the device operated as a Pro Controller.
- PC: powering/resetting while holding the mode-selection button started XInput;
  the controller connected and passed Steam input testing.

## Refactored firmware — 2026-09-13

- Build: `apps/controller/build/esp32-gamepad-controller.bin`
- Flash port: `COM7`
- Flash result: passed with manual BOOT/download and EN/RESET operation.
- Nintendo Switch: connected successfully as a Pro Controller.
- Input result: all tested Switch controller buttons operated normally.
- XInput mode: not yet revalidated after the structural refactor.

## Refactoring acceptance check

Use this list after structural changes:

- [x] Clean build succeeds with ESP-IDF 5.0.1 (2026-09-13, refactored
  `apps/controller`, `esp32-gamepad-controller.bin`).
- [x] Firmware flashes after manually entering download mode (`COM7`, 2026-09-13).
- [ ] Serial log reports successful startup without repeated errors.
- [x] Default boot pairs/reconnects as a Switch Pro Controller (2026-09-13).
- [ ] B-held boot pairs/reconnects as an XInput controller.
- [ ] A-held boot disables CH9350 mouse input.
- [ ] D-pad and A/B/X/Y are correct in both modes.
- [ ] L/R, ZL/ZR, Start/Select, Home/Capture, and stick clicks are correct.
- [ ] Both physical analog sticks move in the expected direction.
- [ ] Touching ZR enables mouse-assisted right-stick control when mouse input is enabled.

Record the tested commit, SDK version, board wiring revision, and any failures below
before starting behavior changes.
