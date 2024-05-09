# HC-12 Firmware Base

## About

You can use this repository as a base to build your own HC-12 firmware.

The radio protocol is compatible with non-modified HC-12 units.

In particular the following works:

* `AT+FU3` settings (including `AT+DEFAULT`)
* setting channel & TX power
* the four modem rates (5kbs-236kbs) are supported

The following does not yet work:

* `FU-1`
* `FU-2` (some efforts have started but are not quite there yet in the
  `fu2-devel` branch)

## Installation

```shell
# install required build tools
sudo apt-get install sdcc build-essential
# grab all submodule dependencies
git submodule update --init --recursive
# build the esp-stlink library
make -C swimcat/esp-stlink/lib
# make the esp-stlink functionality availbale
export PYTHONPATH=$PYTHONPATH:swimcat/esp-stlink/python
```

IMPORTANT: Make sure that your `sdcc --version` is at least 4.4.
There are known compiler bugs in the previous version that will render the
firmware non-functional.

To build the module, just run: `make`

## Flashing

For flashing we use [esp-stlink](https://github.com/rumpeltux/esp-stlink) (pull requests for other flash methods are welcome).

To flash your code to the device run `make flash`.
This will flash the code, but pause execution, so that no output is lost.

To read the console output, run `swimcat/swimcat.py --continue`.

## API

See `si.h` for the available APIs. Notably: sending and receiving radio.
`radio_rx` has to be called after a `radio_tx` to bring the sender into RX state again.

This is linked against the [stm8-arduino library](https://github.com/rumpeltux/stm8-arduino)
for convenience. All its APIs should also be readily usable.

## Demo

The default application (`hc12.c`) implements a simple echo service.
It sends `OpenHC12\r\n` on boot and otherwise resends each packet as received.

This is useful as a communication and range test.

## Restoring the original firmware

For some versions of the chip, you can follow the firmware extraction
instructions in https://github.com/rumpeltux/hc12

## FW Structure

* `hc12.c` is the main application file and making use of [stm8-arduino](https://github.com/rumpeltux/stm8-arduino)
* `si.c` implements the radio interactions.

`si.c` and other libraries that are unlikely to change are bundled to a separate
section of the firmware, so that you don’t need to reflash them all the time
during development.

## Available GPIO PINs

The following STM8 PINs are available for you to use:
A1,A2,A3\* B5(SET) D1\*,D3,D4,D5(TX),D6(RX)

* A3: the original FW used this for signaling chip boot readiness
* D1: SWIM used for programming (already has testport solder pad)

TODO: verify D4 can be used

B5(SET),D5(TX) and D6(RX) already have holes for pin-connectors. \
A3,D3 are at the chip’s edge, so their legs are more easily accessible for soldering.

## Useful references

* [Si4463 Datasheet](https://www.silabs.com/documents/public/data-sheets/Si4464-63-61-60.pdf)
* [Si4463 API Reference (zip)](http://www.silabs.com/documents/public/application-notes/EZRadioPRO_REVB1_API.zip)
* [STM8S Datasheet](https://www.st.com/resource/en/datasheet/stm8s103f2.pdf)
* Pinout (all partially incomplete, but still helpful).
  * https://twitter.com/cathedrow/status/845044463118553091/photo/1
  * https://cxem.net/review/review26.php
