# 	The project root directory, used to locate all other directories.
ROOT_DIR 		:= $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# 	The device to use for flashing the firmware.
# 	Identify the device by following the instructions on:
# 	https://c0-microsd-docs.signaloid.io/guides/identify-c0-microsd
DEVICE			:= /dev/sda

# 	The Python interpreter to use for running the LiteX scripts and toolkit.
PYTHON			:= python3

# 	The path to the Signaloid C0-microSD toolkit for flashing.
TOOLKIT			:= $(ROOT_DIR)/submodules/C0-microSD-utilities/C0_microSD_toolkit.py

# 	The path to the RISC-V compiler toolchain.
CROSS_COMPILE_PATH 	:= /opt/riscv32im/bin/riscv32-unknown-elf
CC			:= $(CROSS_COMPILE_PATH)-gcc
CXX			:= $(CROSS_COMPILE_PATH)-g++
OBJCOPY			:= $(CROSS_COMPILE_PATH)-objcopy

# 	The remove shell command to use for deleting files and directories.
RM			:= rm -rf

# 	The copy shell command to use for copying files and directories.
COPY			:= cp -a

# 	The quiet makefile command to use for silencing shell output.
QUIET      		:= @

# 	Gateware build configuration.
CPU_TYPE		:= vexriscv
CPU_VARIANT		:= lite
SYS_CLK_CFG		:= 12e6
ADD_UART		:= --add_uart
ADD_I2C			:= --add_i2c

# 	Paths configuration.

# 	The path to the Python virtual environment for the project.
VENV_PATH		:= $(ROOT_DIR)/.venv

# 	The path to the LiteX target Python script for the project.
GATEWARE_ROOT_PATH	:= $(ROOT_DIR)/gateware
GATEWARE_SRC_TARGET	:= $(GATEWARE_ROOT_PATH)/signaloid_c0_microsd_target.py

# 	The path to the LiteX bitstream for the project.
GATEWARE_BUILD_PATH 	:= $(ROOT_DIR)/build/signaloid_c0_microsd/gateware
GATEWARE_BITSTREAM	:= $(GATEWARE_BUILD_PATH)/signaloid_c0_microsd.bin

# 	The path to the firmware source directory for the project.
FIRMWARE_ROOT_PATH	:= $(ROOT_DIR)/firmware

# 	The path to the compiled firmware binary for the project.
SOFTWARE_BUILD_PATH 	:= $(ROOT_DIR)/build/signaloid_c0_microsd/software
FIRMWARE_BINARY_NAME	:= signaloid_c0_microsd_firmware
FIRMWARE_BINARY_PATH	:= $(SOFTWARE_BUILD_PATH)/$(FIRMWARE_BINARY_NAME).bin
FIRMWARE_ELF_PATH	:= $(SOFTWARE_BUILD_PATH)/$(FIRMWARE_BINARY_NAME).elf

# 	Documentation build paths.
DOCS_BUILD_PATH 	:= $(ROOT_DIR)/build/documentation
DOCS_BUILD_DIST 	:= $(ROOT_DIR)/build/signaloid_c0_microsd/docs
