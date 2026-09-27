# Validation log

## Verified baseline — 2026-09-13

- Commit: `948c4c2`
- ESP-IDF: 5.0.1
- Result: clean build, flash, and boot passed.
- Nintendo Switch: initial connection required **Controllers → Change Grip/Order**;
  pairing then succeeded and the device operated as a Pro Controller.
- PC: powering/resetting while holding the mode-selection button started XInput;
  the controller connected and passed Steam input testing.

## Refactored firmware — 2026-09-13 to 2026-09-26

- Build: `apps/controller/build/esp32-gamepad-controller.bin`
- Flash port: `COM7`
- Flash result: passed with manual BOOT/download and EN/RESET operation.
- Nintendo Switch: connected successfully as a Pro Controller.
- Input result: all tested Switch controller buttons operated normally.
- XInput mode: revalidated on PC on 2026-09-26. B-held startup connected in
  XInput mode, and normal gameplay was confirmed in *Split Fiction*.

## Refactoring acceptance check

Use this list after structural changes:

- [x] Clean build succeeds with ESP-IDF 5.0.1 (2026-09-13, refactored
  `apps/controller`, `esp32-gamepad-controller.bin`).
- [x] Firmware flashes after manually entering download mode (`COM7`, 2026-09-13).
- [ ] Serial log reports successful startup without repeated errors.
- [x] Default boot pairs/reconnects as a Switch Pro Controller (2026-09-13).
- [x] B-held boot pairs/reconnects as an XInput controller (2026-09-26).
- [ ] A-held boot disables CH9350 mouse input.
- [x] D-pad and A/B/X/Y are correct in both modes.
- [ ] L/R, ZL/ZR, Start/Select, Home/Capture, and stick clicks are correct.
- [x] Both physical analog sticks move in the expected direction in both modes.
- [ ] ZR enables CH9350 aim when mouse input is enabled: native gyro in Switch
  mode (after the game enables IMU), and right-stick control in XInput mode.

Record the tested commit, SDK version, board wiring revision, and any failures below
before starting behavior changes.

## CH9350-to-Switch-gyro preliminary hardware test — 2026-09-26

- User reports Switch mouse gyro aiming felt smooth, but both horizontal and
  vertical directions were reversed and the 160 raw-count sensitivity was too
  high. This was a brief qualitative test, not a full control check.
- The exact flashed build and port were not independently captured. Mouse
  right-button ZR, touch-pedal ZR, and XInput behavior were not specifically
  reported in this test.

## CH9350 gyro adjustment — user Switch test, 2026-09-26

- Build: passed `scripts/build-controller.ps1 -Clean` with ESP-IDF 5.0.1,
  target ESP32, on 2026-09-26. Output:
  `apps/controller/build/esp32-gamepad-controller.bin`.
- Calibration: both gyro axis signs flipped; `BOARD_MOUSE_GYRO_RAW_PER_DELTA`
  reduced from 160 to 80. The adjusted firmware passed another clean build on
  ESP-IDF 5.0.1 / ESP32 on 2026-09-26.
- User reports the adjusted firmware feels smooth on Switch, both directions
  are now correct, and overall sensitivity is closer to the desired value.
  Horizontal screen aiming still feels faster than vertical aiming, described
  as an elliptical response. No measured horizontal/vertical ratio yet.
- Exact flashed build, port, and BOOT/EN sequence were not independently
  captured by the assistant.
- The firmware applies the same raw multiplier (80) to mouse X and Y, so the
  perceived axis difference is not an intentional per-axis gain in the current
  mouse mapping. In-game processing and unimplemented IMU calibration remain
  possible contributors; the cause is not verified.

### Checks still pending

- Confirm flash port `COM7` if another test build is flashed; enter download
  mode manually with BOOT/download and EN/RESET.
- Switch: connect as a Pro Controller, open a game that enables native gyro,
  hold mouse right button to draw a bow, and verify rightward mouse movement
  yaws right and upward movement pitches up. Release the mouse button to fire
  and verify aim stops. Repeat with the original ZR touch pedal, then confirm
  either source can hold ZR independently. Verify the physical right stick still
  works while both ZR sources are released.
- XInput: B-held startup, then verify ZR-held CH9350 right-stick aiming is
  unchanged and the physical right stick works with ZR released.
- Mouse right-button ZR, touch-pedal ZR, and XInput behavior have not yet been
  individually reported for this adjusted firmware.

## Vertical aim tracking report and split-axis calibration — 2026-09-26

- User reports that, while drawing a bow in *Tears of the Kingdom*, tracing a
  vertical pillar with the mouse sometimes makes the crosshair twitch briefly
  left or right and then return. Horizontal tracing in the game is easier to
  keep straight, but is not perfectly straight either; vertical mouse movement
  with an ordinary PC cursor is fairly straight. The source of the Switch-only
  twitch has not been measured, and both axes could be affected differently.
- User also reports horizontal aim is somewhat too fast and vertical aim too
  slow. The next firmware changes only the Switch mouse-to-gyro multipliers:
  yaw 80 to 72 (-10%), pitch 80 to 96 (+20%). XInput mapping is unchanged.
- The split-axis firmware passed `scripts/build-controller.ps1 -Clean` with
  ESP-IDF 5.0.1 for ESP32 on 2026-09-26. Output:
  `apps/controller/build/esp32-gamepad-controller.bin`. It has not been
  flashed or physically retested yet.
- Code inspection found no direct Y-to-yaw mixing. Possible independent
  horizontal inputs include small CH9350 X deltas and the physical right stick,
  which remains active while gyro aiming. Reader/report timing may add jitter
  but cannot by itself create yaw from zero X input. These are hypotheses, not
  hardware diagnoses. Re-test on Switch is pending for the new gains.

## User follow-up on line tracking and sensitivity — 2026-09-26

- After comparing straight-line mouse movement in GTA V on a regular PC, the
  user found that their own vertical and horizontal tracing is not perfectly
  straight there either. The small Switch wobble is currently acceptable in
  play, so no jitter-filtering or sampling change is planned unless it becomes
  disruptive. The exact cause of any remaining wobble has not been measured.
- The user changed the Switch mouse-gyro gains again to yaw 55 and pitch 150
  and reports that this feels more comfortable. These are the current source
  values. The exact flashed build and test sequence were not recorded; a clean
  build from the previous 72/96 source does not validate these newer values.
