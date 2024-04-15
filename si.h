#include <stdint.h>
#include "Arduino.h"

#define SI_IRQ C4
#define SI_IO0 B4
#define SI_IO1 C3
#define SI_CS D2

#define STATE_SI_IRQ PIN4
#define STATE_SI_IO0 PIN5
#define STATE_SI_IO1 PIN6

extern uint8_t interrupt_state;

// initialize the radio. Returns 0 on failure.
uint8_t radio_init(void);

// submits a packet
// for compatibility with original HC-12 devices, make sure len=20
void radio_tx(uint8_t len, const uint8_t *data);

// returns the amount of bytes received, or 0 if no packet is pending
// puts the device into RX state if not already
uint8_t radio_rx(uint8_t len, uint8_t *dest);

void radio_wakeup(void);
void radio_halt(void);

// Internal APIs

// Sets the channel to use. This is using the HC12 numbering,
// i.e. for AT+DEFAULT call si_set_channel(1)
void si_set_channel(uint8_t channel);

// Sets the TX power (0..127).
void si_set_tx_power(uint8_t power);

// Output a character + a number in hex representation.
void si_debug(uint8_t c, uint8_t n);

// Convert a half-byte number (0..15) to a hex character ('0'..'f').
uint8_t si_hex(uint8_t nibble);
