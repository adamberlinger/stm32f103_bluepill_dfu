# STM32F103 Bluepill USB DFU bootloader

This project provides simple and minimalist USB DFU bootloader for STM32F103 bluepill boards.

## Bootloader activation

Bootloader is activated using BOOT pin jumpers on the board. BOOT0=0, BOOT1=1 will activate the bootloader.
There is small part of code that is run after each reset that checks this condition.

## Memory layout

The bootloader is placed in last 4kB of FLASH memory (last 4 pages). So there is no need to remap interrupt vector table in the application firmware. However the application need to avoid using those last 4kB of FLASH.

The bootloader uses a simple trick where it overwrites ResetHandler entry in the NVIC table. So the ResetHandler points to bootloader. Application ResetHandler address is backed up at position 7 (offset 0x1C) which is reserved for Cortex-M3.

Due to this trick, there is a chance that the bootloader will stop working if external reset of the device or board power drop occurs during the programming of the device.

Except for the initial check and RAM initialization, the code runs from internal RAM memory.

## Compiling

To compile the code, Make and arm-none-eabi-gcc toolchain needs to be installed. On Windows I used mingw and [ARM GCC toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)

To compile simply run `make`.

## Programming the bootloader

The bootloader can be loaded either via SWD debug interface. Or via UART system bootloader that can be activated via BOOT pin jumpers: BOOT0=1, BOOT1=0.

When programming the device, it is recommended to enable the write protection in last 4kB in device option bytes.

## Usage

To use the bootloader you can use either [STM32CubeProgrammer](https://st.com/stm32cubeprog) or [DfuSe Demo](https://www.st.com/en/development-tools/stsw-stm32080.html) (deprecated). Although it was mainly tested with STM32CubeProgrammer Please beware that each program requires different USB driver.

## License

The bootloader itself is license under BSD 3-Clause License. It uses CMSIS library for register definition which seems to be BSD 3-Clause License (according to file headers), but latest versions use Apache 2.0 which should be also compatible.