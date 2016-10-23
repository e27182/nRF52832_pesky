# nRF52832_pesky
Programming nRF52832 + MPU9250 + BMP280 dev board from https://www.tindie.com/products/onehorse/nrf52832-development-board/ with nRF52 SDK, GCC and JLink.

Repository contains several examples adapted from official nRF5 SDK. Example folders can contain readme with additional description and video or screenshot of result. Below you can find setup process based on simplest *blinky* example.

##What you will need

1. nRF5 SDK - https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF5-SDK (all code adapted for SDK 12.0.0)
2. ARM GCC Toolchain - https://devzone.nordicsemi.com/tutorials/7/ (I do not use Eclipse yet, just plain GCC, so you could skip part about Eclipse configuration. Also, please, read "How to use it" section before following the tutorial as you need to clone this repo to empty folder, before unpacking SDK there)
3. JLink from SEGGER - I'm using EDU version and have no plans to make it to production
4. MSYS2 - https://msys2.github.io/ (I use Windows as development environment)
  * .bash_profile - you should add JLink and GCC to your PATH, like below

```
PATH="/f/Projects/ARM/gcc-arm-none-eabi-5_4-2016q2-20160622-win32/bin:/c/Program Files (x86)/SEGGER/JLink_V600g:${PATH}"
```

##Features

**make** - compiles code and produces hex and bin files

**make flash** - compiles and flashes hex to board. Also, creates 2 files in _build folder:

  * flash.jlink - commands for JLink used to flash app hex
  * nrf52832_xxaa.hex.log - logfile of the flash process

**make flash_softdevice** - flashes softdevice hex to board. Available only in s132\armgcc\Makefile. Also, creates 2 files in _build folder:

  * flash_softdevice.jlink - command for JLink used to flash softdevice hex
  * s132_nrf52_3.0.0_softdevice.hex.log - logfile of the flash process

##How to use it

*Please review Makefile before executing any command and make sure that you understand what you are doing.*

*All commands below executed in mingw64 shell from msys2.*

1. Clone repository to your *empty* nRF5 SDK folder:

   ```
   mkdir nRF5_SDK_12.0.0_12f24da
   cd nRF5_SDK_12.0.0_12f24da
   git clone https://github.com/e27182/nRF52832_pesky projects
   ```

2. Unpack SDK to the folder, and configure GCC toolchain.
3. Connect your board to JLink.
4. Make and flash. To compile only run *make*, to compile and flash run *make flash*.

   ```
   cd projects\peripheral\blinky\pesky\blank\armgcc
   make flash
   ```

If you wish to use example with SoftDevice s132, you should ensure that you use correct version of soft device. You can flash it with next command:

   ```
   cd projects\peripheral\blinky\pesky\s132\armgcc
   make flash_softdevice
   make flash
   ```

Finally you should see something like that:

![image](https://cloud.githubusercontent.com/assets/14309815/18452651/7c87abb4-7944-11e6-9eff-ac716f8a1380.gif)

## Updates

```
24.10.2016
  Extracted common parts of code into separate folders
  Added Readme.md to uart and md6.12 examples

01.10.2016
  Added UART example
  Added MotionDriver 6.12 adapted example (preliminary version, but working)
    Known issues:
      - could not use NRF_LOG via UART and at the same time use that UART directly
      - need to fix orientation
      - saving sensor calibration values to flash not implemented (commented)

27.09.2016
  Added interrupt example with code to enable pins 9, 10 as ordinary GPIO.
  Added MPU9250 example

13.09.2016
  Added TWI example
  Modified RTT example with timestamping using app_timer

12.09.2016
  Initial commit (blinky)
  Added SEGGER RTT example
```

##TODO

0. Update this readme with video from MD 6.12 example
1. Polish adapted version of Motion Driver 6.12 example
2. Add BLE stack (SoftDevice 132) to all examples
3. Adapt some simple HID device example
4. Create BLE HID joystick example with data from mpu9250
5. Add altimeter data
