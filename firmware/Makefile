# 	Copyright (c) 2024, Signaloid.
#
# 	Permission is hereby granted, free of charge, to any person obtaining a copy
# 	of this software and associated documentation files (the "Software"), to
# 	deal in the Software without restriction, including without limitation the
# 	rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# 	sell copies of the Software, and to permit persons to whom the Software is
# 	furnished to do so, subject to the following conditions:
#
# 	The above copyright notice and this permission notice shall be included in
# 	all copies or substantial portions of the Software.
#
# 	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# 	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# 	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# 	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# 	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# 	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# 	DEALINGS IN THE SOFTWARE.


MAKEFILE_PATH 	:= $(abspath $(firstword $(MAKEFILE_LIST)))
MAKEFILE_DIR 	:= $(dir $(MAKEFILE_PATH))
ROOT_DIR 	:= $(abspath $(MAKEFILE_DIR)/..)


include $(ROOT_DIR)/config.mk
include $(ROOT_DIR)/build/signaloid_c0_microsd/software/include/generated/variables.mak


.PHONY: flash clean print-vars


# 	File paths configuration
SRC_DIR		:= $(FIRMWARE_ROOT_PATH)/src

# 	This is the path to the C0-microSD-litex-integration firmware libraries
SUB_SRC_DIR	:= $(ROOT_DIR)/submodules/C0-microSD-litex-integration/firmware
# 	This is the path to the iCE40-I2C-LiteX-integration firmware libraries
I2C_DRIVER_DIR  := $(wildcard $(VENV_PATH)/lib/python*)/site-packages/iCE40_I2C_LiteX_integration/c_driver_library

LD_DIR		:= $(SOFTWARE_BUILD_PATH)/include/generated
LDSCRIPT	:= $(FIRMWARE_ROOT_PATH)/ld/linker.ld
LDSCRIPTS	:= $(LDSCRIPT) $(LD_DIR)/output_format.ld $(LD_DIR)/regions.ld

CSOURCES	:= $(wildcard $(SRC_DIR)/*.c)
# 	Comment this line if you want to exclude the C0-microSD-litex-integration firmware libraries
CSOURCES	+= $(filter-out $(SUB_SRC_DIR)/src/main.c, $(wildcard $(SUB_SRC_DIR)/src/*.c))
# 	Comment this line if you want to exclude the iCE40-I2C-LiteX-integration firmware libraries
CSOURCES	+= $(wildcard $(I2C_DRIVER_DIR)/*.c)

CPPSOURCES	:= $(wildcard $(SRC_DIR)/*.cpp)
ASOURCES	:= $(wildcard $(SRC_DIR)/*.S)
ASOURCES	+= $(wildcard $(CPU_DIRECTORY)/*.S)

HEADERS		:= $(wildcard $(FIRMWARE_ROOT_PATH)/include/*.h)
# 	Comment this line if you want to exclude the C0-microSD-litex-integration firmware libraries
HEADERS		+= $(wildcard $(SUB_SRC_DIR)/include/*.h)
# 	Comment this line if you want to exclude the iCE40-I2C-LiteX-integration firmware libraries
HEADERS		+= $(wildcard $(I2C_DRIVER_DIR)/*.h)

OBJ_DIR		:= $(SOFTWARE_BUILD_PATH)/.obj

COBJS		:= $(addprefix $(OBJ_DIR)/, $(notdir $(CSOURCES:.c=.o)))
CXXOBJS		:= $(addprefix $(OBJ_DIR)/, $(notdir $(CPPSOURCES:.cpp=.o)))
AOBJS		:= $(addprefix $(OBJ_DIR)/, $(notdir $(ASOURCES:.S=.o)))


# 	Compiler flags configuration
COMMON_FLAGS	:= -I$(FIRMWARE_ROOT_PATH)/include
COMMON_FLAGS	+= -I$(SOFTWARE_BUILD_PATH)/include
COMMON_FLAGS	+= -I$(CPU_DIRECTORY)
COMMON_FLAGS	+= -I$(SOC_DIRECTORY)/software/include
# 	Comment this line if you want to exclude the C0-microSD-litex-integration firmware libraries
COMMON_FLAGS	+= -I$(SUB_SRC_DIR)/include
# 	Comment this line if you want to exclude the iCE40-I2C-LiteX-integration firmware libraries
COMMON_FLAGS	+= -I$(I2C_DRIVER_DIR)
COMMON_FLAGS	+= $(CPUFLAGS)
COMMON_FLAGS	+= -Wall -Wextra
COMMON_FLAGS	+= -ffunction-sections
COMMON_FLAGS	+= -fdata-sections
COMMON_FLAGS	+= -fno-common
COMMON_FLAGS	+= -fomit-frame-pointer
COMMON_FLAGS	+= -Os

CSTD		:= -std=gnu17

CFLAGS		:= $(COMMON_FLAGS)
CFLAGS		+= $(CSTD)

CXXSTD		:= -std=gnu++20

CXXFLAGS	:= $(COMMON_FLAGS)
CXXFLAGS	+= $(CXXSTD)
CXXFLAGS	+= -fno-rtti
CXXFLAGS	+= -fno-exceptions

LFLAGS		:= $(COMMON_FLAGS)
LFLAGS		+= -L$(LD_DIR)
LFLAGS		+= $(CXXSTD)
LFLAGS		+= -nostartfiles
LFLAGS		+= -Wl,--gc-sections
LFLAGS		+= -Wl,--no-warn-mismatch
LFLAGS		+= -Wl,--script=$(LDSCRIPT)
LFLAGS		+= -Wl,--build-id=none
LFLAGS		+= -Wl,--fatal-warnings


# 	Targets
VPATH      := $(SRC_DIR):$(CPU_DIRECTORY)
# 	Comment this line if you want to exclude the C0-microSD-litex-integration firmware libraries
VPATH      += :$(SUB_SRC_DIR)/src
# 	Comment this line if you want to exclude the iCE40-I2C-LiteX-integration firmware libraries
VPATH      += :$(I2C_DRIVER_DIR)


all: $(FIRMWARE_BINARY_PATH)

$(FIRMWARE_BINARY_PATH): $(FIRMWARE_ELF_PATH)
	$(QUIET) echo "  OBJCOPY  $@"
	$(QUIET) $(OBJCOPY) -O binary $(FIRMWARE_ELF_PATH) $@

$(FIRMWARE_ELF_PATH): $(COBJS) $(CXXOBJS) $(AOBJS) $(LDSCRIPTS) $(HEADERS)
	$(QUIET) echo "  LD       $@"
	$(QUIET) $(CXX) $(COBJS) $(CXXOBJS) $(AOBJS) $(LFLAGS) -o $@

$(COBJS): $(OBJ_DIR)/%.o : %.c $(HEADERS)
	$(QUIET) mkdir -p $(OBJ_DIR)
	$(QUIET) echo "  CC       $<	$(notdir $@)"
	$(QUIET) $(CC) -c $< $(CFLAGS) -o $@ -MMD

$(CXXOBJS): $(OBJ_DIR)/%.o: %.cpp $(HEADERS)
	$(QUIET) mkdir -p $(OBJ_DIR)
	$(QUIET) echo "  CXX      $<	$(notdir $@)"
	$(QUIET) $(CXX) -c $< $(CXXFLAGS) -o $@ -MMD

$(AOBJS): $(OBJ_DIR)/%.o: %.S
	$(QUIET) mkdir -p $(OBJ_DIR)
	$(QUIET) echo "  AS       $<	$(notdir $@)"
	$(QUIET) $(CC) -x assembler-with-cpp -c $< $(CFLAGS) -o $@ -MMD


flash: $(FIRMWARE_BINARY_PATH)
	sudo $(PYTHON) $(TOOLKIT) -t $(DEVICE) -b $(FIRMWARE_BINARY_PATH) -u

clean:
	$(QUIET) rm -rf $(OBJ_DIR)
	$(QUIET) echo "  RM       $(OBJ_DIR)"
	$(QUIET) rm -rf $(FIRMWARE_ELF_PATH)
	$(QUIET) echo "  RM       $(FIRMWARE_ELF_PATH)"
	$(QUIET) rm -rf $(FIRMWARE_BINARY_PATH)
	$(QUIET) echo "  RM       $(FIRMWARE_BINARY_PATH)"


print-vars:
	$(foreach v, $(.VARIABLES), $(if $(filter file,$(origin $(v))), $(info $"    - $(v):    $($(v))$")))
