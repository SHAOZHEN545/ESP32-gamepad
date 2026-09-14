# Build and flash

## Known-good environment

- Target: ESP32
- ESP-IDF: 5.0.1
- Baseline commit: `948c4c2`
- Project directory: `apps/controller`

Use an **ESP-IDF 5.0 PowerShell** so that `idf.py`, Python, CMake, Ninja, and the
ESP32 toolchain are already on `PATH`.

## Clean build

From the repository root:

```powershell
cd C:\ESP32-gamepad-project-files\ESP32-gamepad
.\scripts\build-controller.ps1 -Clean
```

The equivalent direct commands are:

```powershell
cd C:\ESP32-gamepad-project-files\ESP32-gamepad\apps\controller
idf.py fullclean
idf.py build
```

On the first build after moving the project, a new `build` directory is expected.
Do not reuse the old `examples/Combo-NS-XBox/build` cache because it contains the
old absolute path.

## Find the serial port

Connect the ESP32 and check **Device Manager → Ports (COM & LPT)**. Note the port
that appears, such as `COM5`.

You can also list likely serial devices in PowerShell:

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name
```

## Flash and monitor

Replace `COM5` with the detected port:

```powershell
.\scripts\flash-controller.ps1 -Port COM5
```

This board does not enter the ESP32 download bootloader automatically. Start the
flash command, then use the board buttons when the terminal shows `Connecting...`:

1. Hold the **BOOT/download** button.
2. Press and release **EN/RESET** while continuing to hold BOOT.
3. Release BOOT after erasing or writing begins.

If this board labels those two buttons differently, use the pair that previously
worked for the verified baseline. This physical action cannot be automated by the
computer.

The script starts the serial monitor after flashing. Press `Ctrl+]` to exit it.

## Switch test

Power or reset the controller without holding A or B. If Switch does not reconnect,
open **Controllers → Change Grip/Order** and put the controller through pairing
there. Test every button, both sticks, ZL/ZR, Home, Capture, and the mouse-assisted
right stick.

## XInput test

Power or reset the controller while holding B, then release B after startup. Pair
it with the PC and test it with Steam's controller input test.
