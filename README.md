# nRF52832_pesky
Programming nRF52832 + MPU9250 + BMP280 dev board from https://www.tindie.com/products/onehorse/nrf52832-development-board/ with nRF52 SDK, GCC and JLink.

Here is what you can do with Invensense Motion Driver SDK 6.12:

![md612 2](https://cloud.githubusercontent.com/assets/14309815/22778555/0aaaa32e-eec0-11e6-9c70-5df3dec03579.gif)

Repository contains several examples adapted from official nRF5 SDK along with extended code with Invensense's Motion Driver 6.12. Example folders can contain readme with additional description and video or screenshot of result, also it contains **hex** subfolder with ready to flash hex file. Below you can find setup process based on simplest *blinky* example.

## List of adapted projects
#### BLE
- [HID Joystick with Motion Driver 6.12](https://github.com/e27182/nRF52832_pesky/tree/master/ble_peripheral/ble_app_hids_joystick_md612)
- [HID Mouse with Motion Driver 6.12](https://github.com/e27182/nRF52832_pesky/tree/master/ble_peripheral/ble_app_hids_mouse_md612)
- [HID Mouse](https://github.com/e27182/nRF52832_pesky/tree/master/ble_peripheral/ble_app_hids_mouse)
- [BLE UART](https://github.com/e27182/nRF52832_pesky/tree/master/ble_peripheral/ble_app_uart)

#### Invensense Motion Driver
- [Invensense Motion Driver Example](https://github.com/e27182/nRF52832_pesky/tree/master/peripheral/md612)
- [MPU9250](https://github.com/e27182/nRF52832_pesky/tree/master/peripheral/mpu9250)

#### Bosch Sensortec BMP280 Driver
- [BMP280](https://github.com/e27182/nRF52832_pesky/tree/master/peripheral/bmp280)

#### Peripherals
- [Blinky](https://github.com/e27182/nRF52832_pesky/tree/master/peripheral/blinky) (detailed description below)
- [Interrupts](https://github.com/e27182/nRF52832_pesky/tree/master/peripheral/pin_change_int)
- [TWI](https://github.com/e27182/nRF52832_pesky/tree/master/peripheral/twi_master_using_app_twi)
- [UART](https://github.com/e27182/nRF52832_pesky/tree/master/peripheral/uart)

#### Segger RTTT
- [Segger RTTT](https://github.com/e27182/nRF52832_pesky/tree/master/segger/rtt)

## What you will need

1. nRF5 SDK - https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF5-SDK (all code adapted for SDK 12.0.0)
2. ARM GCC Toolchain - https://devzone.nordicsemi.com/tutorials/7/ (I do not use Eclipse yet, just plain GCC, so you could skip part about Eclipse configuration. Also, please, read "How to use it" section before following the tutorial as you need to clone this repo to empty folder, before unpacking SDK there)
3. JLink from SEGGER - I'm using EDU version and have no plans to make it to production
4. MSYS2 - https://msys2.github.io/ (I use Windows as development environment)
  * .bash_profile - you should add JLink and GCC to your PATH, like below

```
PATH="/f/Projects/ARM/gcc-arm-none-eabi-5_4-2016q2-20160622-win32/bin:/c/Program Files (x86)/SEGGER/JLink_V600g:${PATH}"
```

## Targets

All standard targets supported, among them:

**make** - compiles code and produces hex and bin files

**make clean** - removes folder with compiled stuff

**make flash** - compiles and flashes hex to board.

**make flash_softdevice** - flashes softdevice hex to board. Available only in s132\armgcc\Makefile.

## How to use it

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
16.01.2018
  Update readme we new goals
  Started updating SDK to newer version
  Investigating moving from modified eMD6.12 to driver provided with [Thingy 52](https://www.nordicsemi.com/eng/Products/Nordic-Thingy-52)
  
19.03.2017
  BLE HID Joystick example added (beta)
  Refactored logging to support UART or RTT, configured with defines in Makefile
  Fixed issue: when board disconnected from J-Link\UART it wont start - added small sleep after TWI initialization

09.02.2017
  Bosch Sensortec BMP280 Driver - peripheral example

30.01.2017
  BLE HID Mouse + MD 6.12 example - board as a bluetooth mouse
  
25.10.2016
  BLE HID Mouse example added
  BLE UART example added
  Hex files added for each example

24.10.2016
  Extracted common parts of code into separate folders
  Added Readme.md to uart and md6.12 examples

01.10.2016
  Added UART example
  Added MotionDriver 6.12 adapted example (preliminary version, but working)
    Known issues:
      - could not use NRF_LOG via UART and at the same time use that UART directly
      - need to fix orientation (fixed 30.01.2017)
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

## TODO

1. Update Nordic Semi SDK to newest version
2. Investigate updating eMD6.12 to eMD provided by NordicSemi with Thingy 52
3. Power-tuning - now I see max 15mA, but, may be, we can do even better?
4. Add DFU support - could be hard due to limited RAM
5. Add altimeter data for 10DOF - could be hard due to limited RAM

## KNOWN ISSUES
1. Windows 10\Edimax BT adapter - both mouse and joystick are unstable, quickly disconnected, lags, etc
> Just add 3.1cm antenna to your board, it should solve the issue.
