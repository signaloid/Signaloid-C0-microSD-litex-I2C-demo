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
ROOT_DIR 	:= $(abspath $(MAKEFILE_DIR))


include $(ROOT_DIR)/config.mk


.PHONY: all prep gateware flash-gateware firmware flash-firmware clean-firmware print-vars-firmware build flash clean clean-env test-target print-vars


all: build

prep: $(VENV_PATH)

$(VENV_PATH):
	git submodule update --init --recursive --remote
	$(PYTHON) -m venv $(VENV_PATH)
	. $(VENV_PATH)/bin/activate && \
	pip install --upgrade pip && \
	pip install -r requirements.txt

gateware: $(GATEWARE_BITSTREAM)

$(GATEWARE_BITSTREAM): $(VENV_PATH) $(GATEWARE_SRC_TARGET)
	. $(VENV_PATH)/bin/activate && \
	$(PYTHON) $(GATEWARE_SRC_TARGET) --cpu-type=$(CPU_TYPE) --cpu-variant=$(CPU_VARIANT) --sys-clk-freq=$(SYS_CLK_CFG) $(ADD_UART) $(ADD_I2C) --build --build_docs && \
	sphinx-build -M html $(DOCS_BUILD_PATH) $(DOCS_BUILD_DIST) && \
	rm -rf $(DOCS_BUILD_PATH)

flash-gateware: $(GATEWARE_BITSTREAM)
	sudo $(PYTHON) $(TOOLKIT) -t $(DEVICE) -b $(GATEWARE_BITSTREAM)


firmware: $(GATEWARE_BITSTREAM)
	$(QUIET) cd $(FIRMWARE_ROOT_PATH) && make --no-print-directory

flash-firmware:
	$(QUIET) cd $(FIRMWARE_ROOT_PATH) && make flash --no-print-directory

print-vars-firmware:
	$(QUIET) cd $(FIRMWARE_ROOT_PATH) && make print-vars --no-print-directory

clean-firmware:
	$(QUIET) cd $(FIRMWARE_ROOT_PATH) && make clean --no-print-directory


build: gateware firmware

flash: flash-gateware flash-firmware

clean:
	$(QUIET) rm -rf ./build
	$(QUIET) echo "  RM       ./build"
	$(QUIET) rm -rf ./__pycache__
	$(QUIET) echo "  RM       ./__pycache__"

clean-env:
	rm -rf $(VENV_PATH)

test-target:
	. $(VENV_PATH)/bin/activate && \
	python -m gateware.signaloid_c0_microsd_target --cpu-type=$(CPU_TYPE) --cpu-variant=$(CPU_VARIANT) --sys-clk-freq=$(SYS_CLK_CFG) $(ADD_UART) --build --no-compile

print-vars:
	$(foreach v, $(.VARIABLES), $(if $(filter file,$(origin $(v))), $(info $"    - $(v):    $($(v))$")))
