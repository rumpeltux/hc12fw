# HC-12 Firmware Base

## EXPERIMENTAL / UNFINISHED [DEVELOPMENT BRANCH]

The code in this repository is absolutely experimental and may not work for you!
Since I stopped active development, I’m releasing this WIP version for others to
pick up.

This is the development branch for FU2 support. IT IS NOT YET WORKING.
The code is not cleaned up and full of debug statements (sorry).

## About

You can use this repository as a base to build your own HC-12 firmware.

The radio protocol is compatible with `AT+DEFAULT` settings in non-modified
HC-12 units assuming that you use a packet-length of 20 bytes. For HC-12 to
receive packets, you need to use a 2-byte header: [0x18, payload-length].

None of the other communication modes are implemented for now, but you can
change channel and TX power.

Some efforts towards FU-2 have started but are not quite there yet in the
`fu2-devel` branch.

## Dependencies

https://github.com/ZakKemble/Si446x (for Si446x/Si446x/Si446x_defs.h)
https://github.com/rumpeltux/stm8-arduino
https://github.com/rumpeltux/stm8-swimcat

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
* RFIC Programming Guide:
  http://www.farnell.com/datasheets/1889753.pdf
