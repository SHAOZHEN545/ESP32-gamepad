# References

Sources used while designing and building the controller.

## Firmware and tutorials

- [HOJA-LIB-ESP32](https://github.com/HandHeldLegend/HOJA-LIB-ESP32): the
  controller library this repository forks.
- Mundo Yakara's [ESP32 online flashing tool](https://yakaracolombia.github.io/esp32-online-tool/arcade2.html)
  for a Bluetooth arcade stick, built on HOJA. Write-ups:
  [Bluetooth arcade control](https://www.mundoyakara.com/2022/07/hacer-control-arcade-bluetooth.html),
  [arcade control for Switch](https://www.mundoyakara.com/2022/07/hacer-control-de-arcade-para-switch-con.html).
- [Soldering and software walkthrough](https://www.youtube.com/watch?v=NLdcXWorapA)
  for that project. Useful timestamps: 2:22 default pin map, 3:55 the flashing
  site, 6:37 hold BOOT while flashing, 25:49 automatic Bluetooth reconnection,
  31:06 cutting the panel, 44:47 BlueRetro advanced settings for custom pins.
- [r/esp32 thread](https://www.reddit.com/r/esp32/comments/1hbeuud/this_is_just_for_people_that_wasted_as_much_time/)
  on common ESP32 setup pitfalls.
- [Using a PS4 controller with an ESP32 over Bluetooth](https://www.youtube.com/watch?v=EEViXFoSzww).

## Similar projects

- [nxbt](https://github.com/Brikwerk/nxbt): control a Switch from a website,
  terminal, or macro.
- [joycontrol](https://github.com/mart1nro/joycontrol): emulate Switch
  controllers over Bluetooth.
- [Dual analog lever and thumbstick arcade stick](https://www.reddit.com/r/fightsticks/comments/wp8hbj/dual_analog_lever_thumbstick_arcade_stick_proof/):
  a proof of concept close to this project's idea, with a good layout.

## Parts

- Digital arcade stick (Sanwa-style):
  [AliExpress](https://www.aliexpress.us/item/3256807629501218.html)
- Arcade buttons (Sanwa-style):
  [AliExpress](https://www.aliexpress.us/item/3256807858720487.html)
- Analog joysticks: [JH-D202X product page](https://protosupplies.com/product/jh-d202x-r2-r4-joystick-5k/)
  (related series), [Taobao listing](https://goods.taobao.com/t/dianweiqi_11081/ac3a745ee0ac31603a3b500da2d8d6bd.html),
  [Taobao shop](https://world.taobao.com/dianpu/192295753.htm),
  [video](https://www.youtube.com/watch?v=uUeDE8o02q8).
  Mounting-hole dimensions: [PRZ R200B-M1](https://www.micros.com.pl/en/product/prz-r200b-m1,257256.html).
  CAD model: [3-axis joystick with button](https://grabcad.com/library/3-axis-joystick-w-button-1).
- CH9350 USB-host module: [video](https://www.youtube.com/watch?v=kLLIgxpmcA8),
  [Taobao listing](https://vn.world.taobao.com/item/695173316772.htm).
- Enclosure: [MakerCase box generator](https://www.makercase.com/basicBox),
  [0.118 in polycarbonate sheet](https://www.grainger.com/product/Polycarbonate-Sheet-0-118-1ETL1).
