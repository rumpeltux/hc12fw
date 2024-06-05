# The path at which the stm8-arduino lib is located
ARDUINO ?= arduino

# The default target to build. Can be overridden, e.g. `make TARGET=range_test_demo`
TARGET ?= echo_demo
FLASH_CMD ?= swimcat/esp-stlink/python/flash.py
FLASH_ARGS ?= --stall

# For v2.3/v2.4 set this to 24
REVISION ?= 26

CC := sdcc
CFLAGS := -mstm8 --std-c99 --opt-code-size -I$(ARDUINO)/include -L$(ARDUINO)/src -DSWIMCAT_BUFSIZE_BITS=7 -DREVISION=$(REVISION)
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
	  [ -e $$i.needsflash ] && $(FLASH_CMD) $(FLASH_ARGS) -i $$i && rm $$i.needsflash || true; \
	done

clean:
	$(MAKE) -C arduino clean
	$(MAKE) -C swimcat clean
	rm -f *.asm *.cdb *.ihx *.lnk *.lk *.lst *.map *.mem *.rel *.rst *.sym *.needsflash static.lib.*
