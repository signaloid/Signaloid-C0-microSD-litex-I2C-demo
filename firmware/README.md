# Signaloid C0-microSD Firmware: I2C interface
This is an example C based firmware for the extended target design of the Signaloid C0-microSD with I2C interface, based on the default target design defined in the [LiteX-Boards](https://github.com/litex-hub/litex-boards) repository.

The firmware implements an I2C example, which reads the raw IR & Red LED data from a MAX30100 Pulse/Oxymeter sensor, and displays them on an SSD1306 128x32 OLED display. All data is communicated over I2C. Also, application flow, all sensor data, and LED states are printed on UART.

For the purposes of this project, we modified the following repositories to work on our setup:
- [LibDriver SSD1306 OLED Display driver](https://github.com/libdriver/ssd1306):
	- Implemented the interface file.
	- Adapted the driver so it can handle both 128x64 and 128x32 resolution displays.
	- Optimized the driver example so it updates the display only on request, not on every draw.
- [xcoder123 MAX30100 Pulse/Oxymeter driver](https://github.com/xcoder123/MAX30100):
	- Removed/Replaced all Arduino related libraries.
	- Created the init function to implement the class constructor functionality, so it can initialize the static MAX30100 object.

## Dependencies
Building the firmware requires that you have already installed the [RISC-V GNU Toolchain](https://github.com/riscv/riscv-gnu-toolchain) for RV32IM ISA extension. It assumes that it is installed in the default location, `opt/riscv32im/bin` on the host machine. If this is not the case, you can change its path in the `config.mk` file, on the `CROSS_COMPILE_PATH` variable. Also, make sure that the `DEVICE` variable in the `config.mk` file is set to the correct device path (follow the [Identify the Signaloid C0-microSD](https://c0-microsd-docs.signaloid.io/guides/identify-c0-microsd) guide).

## Getting Started
### Prerequisites
You must have already built the Signaloid C0-microSD gateware for the firmware to reference hardware support libraries. If you have already built the gateware, you can skip this step. To build the gateware and supporting libraries you can run the following commands:

Prepare the environment, if needed, and build the gateware. Run this in the project's root directory.
```sh
make gateware
```

### Makefiles
There is a dedicated Makefile for the firmware needs, which is located in the `firmware/` directory. It is included in the main Makefile, so you can run the commands from both the `firmware/` and the project's root directory.

Build the firmware. Run this in the project's `firmware/` directory.
```sh
make
```
or  
Run this in the project's root directory.
```sh
make firmware
```

Flash the firmware. This also builds the firmware, if there are changes. Make sure that the `DEVICE` variable in the `config.mk` file is set to the correct device path. Run this in the project's `firmware/` directory.
```sh
make flash
```
or  
Run this in the project's root directory.
```sh
make flash-firmware
```

Clean the build files. Run this in the project's `firmware/` directory.
```sh
make clean
```
or  
Run this in the project's root directory.
```sh
make clean-firmware
```

Print the variables used in the Makefile. Run this in the project's `firmware/` directory.
```sh
make print-vars
```
or  
Run this in the project's root directory.
```sh
make print-vars-firmware
```

## Firmware Binary
The firmware binary is stored in the `build/signaloid_c0_microsd/software/` directory with the name `signaloid_c0_microsd_firmware.bin`.

## Serial
The serial port is used for communication with the Signaloid C0-microSD. It is connected to the Signaloid C0-microSD's platform serial pins. Serial communication can be configured in the following way:

**Baudrate:** 115200

**Pins used:**
- `tx`=`A4|SD_CMD`
- `rx`=`B3|SD_CLK`

Serial communication can be accessed using `screen` on linux, or other similar tools.
```sh
screen /dev/ttyACM0 115200
```

## Execution description
The firmware binary is flashed using the Signaloid C0-microSD-toolkit through `make flash` or `make flash-firmware` on a specific address (USER_DATA_OFFSET: 0x200000) of the on-board SPI Flash (see [Bootloader Addressing](https://c0-microsd-docs.signaloid.io/hardware-overview/bootloader-addresssing.html)). This address is set as the `cpu_reset_address` in the target design script. Hence, when the Signaloid C0-microSD bitstream is loaded by the bootloader, the SoC will start executing from this address.

The entry point of the Signaloid C0-microSD firmware is the `_start` symbol, as defined by `linker.ld`. The `_start` symbol is defined in `build/signaloid_c0_microsd/software/core_libs/crt0.S`. `crt0.s` is an assembly file located at the path defined in the `CPU_DIRECTORY` variable from the `build/signaloid_c0_microsd/software/include/generated/variables.mak` file. It is responsible for:
- Setting the stack pointer
- Initializing the isr and exception vectors
- Initializing the global pointer
- Initializing the data section, by copying the rodata section
- Initializing the bss section
- Calling the C startup routine `main`

The execution continues as normal on the `main` function.

> [!NOTE]
> There is no instruction caching in this design. The whole SRAM (128kiB) is used for the data section of the application. All instructions are sequentially fetched from the on-board SPI Flash.
