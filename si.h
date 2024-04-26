#include "Arduino.h"
#include <stdint.h>

#define SI_IRQ C4
#define SI_IO0_TXSTATE B4
#define SI_IO1_CTS C3
#define SI_CS D2

#define SI_IRQ_RECEIVING 1

#define STATE_SI_IO0 PIN5
#define STATE_SI_IO1 PIN6

#define SI_STATE_NO_CHANGE 0
#define SI_STATE_SLEEP 1
#define SI_STATE_READY 3
#define SI_STATE_TX_TUNE 5
#define SI_STATE_RX_TUNE 6
#define SI_STATE_TX 7
#define SI_STATE_RX 8

// initialize the radio. Returns 0 on failure.
uint8_t radio_init(void);

// submits a packet
// for compatibility with original HC-12 devices, make sure len=20
void radio_tx(uint8_t len, const uint8_t *data);

// Sets up the device to receive a packet of len bytes.
void radio_start_rx(uint8_t len);

// Receives a packet of len. Blocks until a packet is received.
// May return 0 if the packet is invalid (e.g. CRC failure).
// Puts the device into RX state if not already
// dest must be at least min(8, len) bytes long.
uint8_t radio_rx(uint8_t len, uint8_t *dest);

void radio_wakeup(void);
void radio_halt(void);

// Internal APIs

// Sets the channel to use. This is using the HC12 numbering,
// i.e. for AT+DEFAULT call si_set_channel(1)
void si_set_channel(uint8_t channel);

// Sets the TX power (0..127).
void si_set_tx_power(uint8_t power);

// returns -1 on failure
int8_t si_get_rx_fifo_size(void);

// Output a character + a number in hex representation.
void si_debug(uint8_t c, uint8_t n);

// Convert a half-byte number (0..15) to a hex character ('0'..'f').
uint8_t si_hex(uint8_t nibble);

// blocks in wfi() until si_notify_packet_irq is called.
// returns the interrupt status flags (0x10: packet received, 0x08: CRC error)
uint8_t si_wait_packet(void);
void si_notify_nirq(void);

void si_wait_radio_tx_done(void);

// Returns the current device state (see SI_STATE_…).
uint8_t si_get_state(void);
