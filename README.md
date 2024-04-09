# HC-12 Firmware Base

## EXPERIMENTAL / UNFINISHED

The code in this repository is absolutely experimental and may not work for you!
Since I stopped active development, I’m releasing this WIP version for others to
pick up.

## About

You can use this repository as a base to build your own HC-12 firmware.

The radio protocol is compatible with `AT+DEFAULT` settings in non-modified
HC-12 units assuming that you use a packet-length of 20 bytes. For HC-12 to
receive packets, you need to use a 2-byte header: [0x18, payload-length].

None of the other communication modes are implemented for now, but you can
change channel and TX power.

Some efforts towards FU-2 have started but are not quite there yet in the
`fu2-devel` branch.

## Installation

```shell
# install required build tools
sudo apt-get install sdcc make
# grab all submodule dependencies
git submodule update --init --recursive
# build the esp-stlink library
make -C swimcat/esp-stlink/lib
# make the esp-stlink functionality availbale
export PYTHONPATH=$PYTHONPATH:swimcat/esp-stlink/python
```

To build the module, just run: `make`

## Flashing

For flashing we use [esp-stlink](https://github.com/rumpeltux/esp-stlink) (pull requests for other flash methods are welcome).

To flash your code to the device run `make flash`.
This will flash the code, but pause execution, so that no output is lost.

To read the console output, run `swimcat/swimcat.py --continue`.

## API

See `si.h` for the available APIs. Notably: sending and receiving radio.
`radio_rx` has to be called after a `radio_tx` to bring the sender into RX state again. TODO: verify why…

This is linked against the [stm8-arduino library](https://github.com/rumpeltux/stm8-arduino)
for convenience. All its APIs should also be readily usable.

## Demo

The default application implements a simple echo service.
It sends `Hello\r\n` on boot and otherwise resends each packet as received.

## Restoring the original firmware

To create a backup that you can restore, follow the firmware extraction
instructions in https://github.com/rumpeltux/hc12

## FW Structure
Radio interrupts. Interrupt handler sets flags:

* RX pending
* TX complete

Main loop:

TODO

## Available GPIO PINs

The following STM8 PINs are available for you to use:
A1,A2,A3\* B5(SET) D1\*,D3,D4,D5(TX),D6(RX)

* A3: signaling chip boot readiness (TODO: verify if this is needed)
* D1: SWIM used for programming (already has testport solder pad)

TODO: verify D4 can be used

B5,D5 and D6 already have holes for pin-connectors. \
A3,D3 are at the chip’s edge, so their legs are more easily accessible for soldering.

## Useful references

* [Si4463 Datasheet](https://www.silabs.com/documents/public/data-sheets/Si4464-63-61-60.pdf)
* [STM8S Datasheet](https://www.st.com/resource/en/datasheet/stm8s103f2.pdf)
* Pinout (all partially incomplete, but still helpful).
  * https://twitter.com/cathedrow/status/845044463118553091/photo/1
  * https://cxem.net/review/review26.php
