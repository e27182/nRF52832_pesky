# nRF52832_pesky
Programming nRF52832 + MPU9250 + BMP280 dev board from https://www.tindie.com/products/onehorse/nrf52832-development-board/ with nRF52 SDK, GCC and JLink.

##Prerequisites

1. nRF5 SDK - https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF5-SDK
2. ARM GCC Toolchain - https://devzone.nordicsemi.com/tutorials/7/ (I do not use Eclipse yet, just plain GCC, so you could skip part about Eclipse configuration)
3. JLink from SEGGER - I'm using EDU version and have no plans to make it to production
4. MSYS2 - https://msys2.github.io/ (I use Windows as development environment)
..* .bash_profile - you should add JLink and GCC to you PATH, like below

```
PATH="/f/Projects/ARM/gcc-arm-none-eabi-5_4-2016q2-20160622-win32/bin:/c/Program Files (x86)/SEGGER/JLink_V600g:${PATH}"
```

##How to use it

1. Clone repository to your nRF5 SDK folder:

```
cd nRF5_SDK_12.0.0_12f24da
git clone https://github.com/e27182/nRF52832_pesky/projects
```

2. Connect your board to JLink
3. Make and flush

```
cd projects/blinky...
```

4. 
