# Controller firmware

This is the actively maintained firmware for the custom ESP32 control box.

At startup it defaults to Nintendo Switch mode with CH9350 mouse input enabled.
The direct GPIO buttons are active low:

- Hold **B** while powering on or resetting to start in Bluetooth XInput mode.
- Hold **A** while powering on or resetting to disable mouse input.
- Hold **A+B** to select XInput mode with mouse input disabled.
- Release the held buttons when the startup log asks the controller to continue.

The first Switch connection may require opening **Controllers → Change
Grip/Order**. After pairing, the device appears as a Pro Controller.

See [`../../docs/build-and-flash.md`](../../docs/build-and-flash.md) for the full
build and flash procedure and [`../../docs/hardware.md`](../../docs/hardware.md)
for the current pin map.

## Source layout

- `app_main.c`: startup-mode selection and HOJA startup.
- `controller_input.c`: converts board input into HOJA button and analog state.
- `analog_sticks.c`: ADC setup, calibration, dead zones, and stick mapping.
- `button_expander.c`: TCA9555 setup and button reads.
- `touch_triggers.c`: ZL/ZR capacitive calibration and reads.
- `mouse_input.c`: CH9350 UART parsing and mouse-to-right-stick mapping.
- `board_config.h`: board pins and device-level constants.
