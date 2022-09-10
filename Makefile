#
# BSD 3-Clause License
# 
# Copyright (c) 2022, Adam Berlinger
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

TARGET_NAME=stm32f103_bluepill_dfu

# Initial variable setup
BUILD_DIR:=build
SOURCE_DIRS=src stm32x0_usb_dfu
GLOBAL_DEPS=Makefile
LINKER_SCRIPT=src/linkerscript.ld
INCLUDE_PATH=src stm32x0_usb_dfu $(BUILD_DIR)/$(TARGET_NAME)/generated_files

# ARM specifics
TOOL_PREFIX=arm-none-eabi-
INCLUDE_PATH+=lib/CMSIS/Include

# F1 specifics
C_MACROS+=STM32F103xB STM32F1XX
INCLUDE_PATH+=lib/CMSIS/Device/ST/STM32F1xx/Include
OPTS+=-mcpu=cortex-m3 -march=armv7-m -mthumb -Wno-main

# Export all source files from source directories
SOURCES=$(wildcard $(addsuffix /*.c,$(SOURCE_DIRS)))
HEADERS=$(wildcard $(addsuffix /*.h,$(INCLUDE_PATH)))
ASSEMBLERS=$(wildcard $(addsuffix /*.s,$(SOURCE_DIRS)))

# Export source files for supported peripherals
SOURCES+=$(PERIPHERALS:%=api/%.c)
PERIPH_MACROS=$(shell echo $(PERIPHERALS) | tr a-z A-Z)
PERIPH_MACROS+=$(shell echo $(PERIPHERALS_CPP) | tr a-z A-Z)

# Export define macros for supported peripherals
C_MACROS+=$(PERIPH_MACROS:%=%_PERIPH_ENABLED)
C_MACROS+=TARGET_NAME=\"$(TARGET_NAME)\"

# Toolchain setup
CC=$(TOOL_PREFIX)gcc
LD=$(TOOL_PREFIX)ld
OBJCOPY=$(TOOL_PREFIX)objcopy
GDB=$(TOOL_PREFIX)gdb

# Export compiler options: optimization, include path and C macros
CCOPTS:=$(OPTS)
OPTS+=-Wall -g -Os -nostartfiles
OPTS+=$(INCLUDE_PATH:%=-I%)
OPTS+=$(C_MACROS:%=-D%)

OPTS+=-fgnu89-inline

# Create path for build output
TARGET_DIR=$(BUILD_DIR)/$(TARGET_NAME)
OBJECTS=$(SOURCES:%.c=$(TARGET_DIR)/%.c.o)
OBJECTS+=$(ASSEMBLERS:%.s=$(TARGET_DIR)/%.s.o)
STATS=$(SOURCES:%.c=$(TARGET_DIR)/stats/%.stats)
ELF_FILE=$(TARGET_DIR)/build.elf
HEX_FILE=$(TARGET_DIR)/$(TARGET_NAME).hex

# Configuration of GDB server
GDB_SERVER=openocd -f target/$(TARGET_NAME)/openocd.cfg

.PHONY: debug flash elf doc

hex: $(HEX_FILE)

elf: $(ELF_FILE)

flash: $(ELF_FILE)
	$(GDB_SERVER) -c "program $(ELF_FILE) verify reset"

server:
	$(GDB_SERVER)

debug: $(ELF_FILE)
	$(GDB) $(ELF_FILE) -x startup.gdb

$(HEX_FILE): $(ELF_FILE)
	$(OBJCOPY) -O ihex $(ELF_FILE) $(TARGET_DIR)/$(TARGET_NAME).hex

-include $(OBJECTS:.o=.d)

$(ELF_FILE): $(OBJECTS) $(LINKER_SCRIPT)
	@echo "Linking..."
	@$(CC) $(OPTS) -Wl,-M -T $(LINKER_SCRIPT) $(OBJECTS) -o $(ELF_FILE) > $(TARGET_DIR)/build.map
	@echo "Program '$(TARGET_NAME)' builded"

$(TARGET_DIR)/%.c.o: %.c $(GLOBAL_DEPS)
	@mkdir -p ${@D}
	@echo "Compiling $*.c ..."
	@$(CC) -c $(OPTS) $*.c -o $(TARGET_DIR)/$*.c.o
	@$(CC) -MM -MP -MT $@ $(OPTS) $*.c > $(TARGET_DIR)/$*.c.d

$(TARGET_DIR)/%.s.o: %.s $(GLOBAL_DEPS)
		@mkdir -p ${@D}
		@echo "Compiling $*.s ..."
		@$(CC) -c $(OPTS) $*.s -o $(TARGET_DIR)/$*.s.o

clean:
	rm -rf $(TARGET_DIR)/*
