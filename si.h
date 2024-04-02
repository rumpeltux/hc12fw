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

// flag cleared when reading fifo or starting new RX
#define INTERRUPT_STATE_RX_READY 1
// flag cleared when starting new RX
#define INTERRUPT_STATE_RX_ERROR 2
#define INTERRUPT_STATE_TX_COMPLETE 4

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
void fu2_start_rx(uint8_t long_mode);
uint8_t si_read_fifo(uint8_t len, uint8_t *dest);
uint8_t fu2_radio_rx(uint8_t len, uint8_t *dest);
void si_stop_rx();
void si_interrupt_handler();
uint8_t fu2_radio_rx_long(uint8_t len, uint8_t *dest);
uint8_t si_get_state();
extern void si_read_interrupt_status2(uint8_t *buf);


uint8_t radio_rx(uint8_t len, uint8_t *dest);
void debug(uint8_t c, uint8_t n);

void radio_wakeup();
void radio_halt();
