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
extern uint8_t si_latched_rssi;
extern uint8_t radio_buf[0x40];

// initialize the radio.
void init_radio();

// submits a packet
// for compatibility with original HC-12 devices, make sure len=20
void radio_tx(uint8_t len, const uint8_t *data);

// TODO: check if this blocks until a packet has arrived
// returns the amount of bytes received
void hexout16(uint16_t data);
void hexout(uint8_t data);
uint8_t fu2_radio_rx(uint8_t len, uint8_t *dest);
uint8_t fu2_radio_rx_long(uint8_t len, uint8_t *dest);


uint8_t radio_rx(uint8_t len, uint8_t *dest);
void debug(uint8_t c, uint8_t n);

void radio_wakeup();
void radio_halt();
