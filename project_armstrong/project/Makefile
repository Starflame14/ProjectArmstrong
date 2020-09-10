# Copyright (c) 2019-2020, Arm Limited. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

CROSS_COMPILE?=arm-none-eabi-
TARGET += -mcpu=cortex-m0 -mthumb -mfloat-abi=softfp

CFLAGS += -g2 -O0 -Wall -Werror -fno-strict-aliasing
CXXFLAGS += -g2 -O0 -Wall -Werror -fno-strict-aliasing -fno-exceptions -fno-rtti

BUILDDIR := build

SOURCES := $(shell find . -name "*.c")
SOURCES += $(shell find . -name "*.S")

INCLUDE_DIRS := .

OBJS := $(SOURCES:.c=.o)
OBJS := $(OBJS:.S=.o)
OBJS := $(addprefix $(BUILDDIR)/,$(OBJS))
DEPS := $(OBJS:.o=.d)

LIBS += m
LIBS += c

LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs

LDSCRIPT := fvp.ld

# Compiler flags
CFLAGS += $(TARGET)
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += $(addprefix -D,$(DEFINES))
CFLAGS += $(addprefix -I,$(INCLUDE_DIRS))

CXXFLAGS += $(TARGET)
CXXFLAGS += -fdata-sections -ffunction-sections
CXXFLAGS += $(addprefix -D,$(DEFINES))
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRS))

ASFLAGS += $(TARGET)

LDFLAGS += $(TARGET)
LDFLAGS += $(addprefix -L,$(LIB_DIRS))
LDFLAGS += $(addprefix -l,$(LIBS))

CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)g++
LD=$(CROSS_COMPILE)g++
OBJDUMP=$(CROSS_COMPILE)objdump
OBJCOPY=$(CROSS_COMPILE)objcopy

MKDIR:=mkdir -p

all: rocket

clean:
	rm -rf $(BUILDDIR) rocket.elf rocket.bin

rocket: rocket.elf rocket.bin $(BUILDDIR)/rocket.dump

$(BUILDDIR)/rocket.dump: rocket.elf
	@echo "Exporting  $(@)"
	@$(OBJDUMP) -d $(<) > $(@)

rocket.bin: rocket.elf
	@echo "Creating   $(@)"
	@$(OBJCOPY) -O binary $(<) $(@)

rocket.elf: $(OBJS)
	@echo "Linking    $(@)"
	@$(LD) $(LDFLAGS) -T $(LDSCRIPT) $(OBJS) $(STATIC_LIBS) -o $@

$(BUILDDIR)/%.o: %.c
	@echo "CC  $(<)"
	@$(MKDIR) $(dir $(@))
	@$(CC) $(CFLAGS) -MMD -MP -MF$(@:%.o=%.d) -MT$(@) -c $(<) -o $(@)

$(BUILDDIR)/%.o: %.S
	@echo "AS $(<)"
	@$(MKDIR) $(dir $(@))
	@$(AS) $(ASFLAGS) -c $(<) -o $(@)

-include $(DEPS)
