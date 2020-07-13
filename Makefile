# The path at which the stm8-arduino lib is located
ARDUINO ?= arduino

# The stm8flash command used to flash the target, you can override it here or via
# $ export STM8FLASH=stm8flash...
STM8FLASH ?= stm8flash -c espstlink -p stm8s103?3

# The default target to build.
TARGET ?= hc12

CC := sdcc
CFLAGS := -mstm8 --std-c99 --opt-code-size -I$(ARDUINO)/include -L$(ARDUINO)/src
ARDUINO_LIB := $(ARDUINO)/src/arduino.lib

all: $(TARGET).ihx

%.rel: %.o %.c
	@echo -n

$(ARDUINO_LIB): $(ARDUINO)/src/*
	make -C $(ARDUINO)/src

# If you have dependencies add:
hc12.ihx: si.rel

$(TARGET).ihx: $(ARDUINO_LIB) $(TARGET).rel
	$(CC) $(CFLAGS) -larduino $(filter-out $<,$^)

flash: $(TARGET).ihx
	$(STM8FLASH) -w $<

clean:
	rm -f *.asm *.cdb *.ihx *.lnk *.lk *.lst *.map *.mem *.rel *.rst *.sym
