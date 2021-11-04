# The path at which the stm8-arduino lib is located
ARDUINO ?= arduino

# The default target to build.
TARGET ?= hc12

CC := sdcc
CFLAGS := -mstm8 --std-c99 --opt-code-size -I$(ARDUINO)/include -L$(ARDUINO)/src
ARDUINO_LIB := $(ARDUINO)/src/arduino.lib

all: $(TARGET).ihx

%.rel: %.o %.c
	@echo -n
%.rel: %.S
	sdasstm8 -lo $<

$(ARDUINO_LIB): $(ARDUINO)/src/*
	make -C $(ARDUINO)/src

static.lib.S: mklib.py static.lib.ihx
	python3 mklib.py static.lib.map > $@

static.lib.ihx: si.rel swimcat/swimcat.rel

static.lib.ihx: $(ARDUINO_LIB)
	$(CC) $(CFLAGS) -larduino $(filter-out $<,$^) --code-loc 0x9000 --stack-loc 0x400 -o $@
	touch $@.needsflash

$(TARGET).ihx: $(ARDUINO_LIB) $(TARGET).rel static.lib.rel $(ARDUINO)/src/main.rel
	$(CC) $(CFLAGS) -larduino $(filter-out $<,$^) --data-loc $$(cat static.lib.datastart)
	touch $@.needsflash

flash: $(TARGET).ihx static.lib.ihx
	for i in $^; do \
	  [ -e $$i.needsflash ] && esp-stlink/python/flash.py --stall -i $$i; \
	done

clean:
	rm -f *.asm *.cdb *.ihx *.lnk *.lk *.lst *.map *.mem *.rel *.rst *.sym *.needsflash static.lib.*
