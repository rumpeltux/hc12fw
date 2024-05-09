#include "Arduino.h"
#include <stdint.h>

#define SI_IRQ C4
#define SI_IO0_TXSTATE B4
#define SI_IO1_CTS C3
#define SI_CS D2

extern const uint8_t si_config_5kbit[];
extern const uint8_t si_config_15kbit[];
extern const uint8_t si_config_58kbit[];
extern const uint8_t si_config_236kbit[];

// Initialize the radio. Returns 0 on failure.
uint8_t radio_init(const uint8_t *si_config_p);

// Submits a packet
// For compatibility with original HC-12 devices, make sure to use the
// appropriate HC12_PACKET_SIZE constant.
void radio_tx(uint8_t len, const uint8_t *data);

// Retrieves len bytes of data from the current packet.
// Blocks until a packet is received.
// Starts the receiver (`radio_start_rx()`) if not yet started.
// May return 0 if the packet is invalid (e.g. CRC failure) or does not
// contain at least len bytes.
// dest must be at least min(8, len) bytes long.
uint8_t radio_rx(uint8_t len, uint8_t *dest);

// Puts the radio in sleep state for low power consumption.
void radio_halt(void);

// Sets the channel to use. This is using the HC12 numbering,
// i.e. for AT+DEFAULT call si_set_channel(1)
void si_set_channel(uint8_t channel);

// Sets the TX power (0..127).
void si_set_tx_power(uint8_t power);

// Sets up the radio to receive a packet of len bytes.
// len=0 indicates a variable length packet. The default settings
// configure the first byte as length.
void si_start_rx(uint8_t len);

// blocks in wfi() until si_notify_nirq is called and one of the
// interrupt flags (0x10: packet received, 0x08: CRC error) is set.
// returns the interrupt status flags
uint8_t si_wait_packet(void);

// blocks in wfi() until si_notify_nirq is called and the
// PACKET_SENT interrupt flag is set.
void si_wait_radio_tx_done(void);

// Notifies the si radio library that the NIRQ pin was triggered.
// This should be called from a respective interrupt handler on
// the GPIO. It is not timing sensitive, as the NIRQ pin stays low
// until interrupts have been cleared by software.
void si_notify_nirq(void);

// Sends a set of radio params to the device. Preconfigured options are listed below.
void si_radio_config(const uint8_t *si_config_p);

#define SI_STATE_NO_CHANGE 0
#define SI_STATE_SLEEP 1
#define SI_STATE_READY 3
#define SI_STATE_TX_TUNE 5
#define SI_STATE_RX_TUNE 6
#define SI_STATE_TX 7
#define SI_STATE_RX 8

// Returns the current device state (see SI_STATE_…).
uint8_t si_get_state(void);

// Lower level internal APIs

// clears RX & TX fifos
void si_clear_fifo(void);

// returns -1 on failure
int8_t si_get_rx_fifo_size(void);

void si_read_rx_fifo(uint8_t len, uint8_t *dest);

// Some debug utilities.

// Output a character + a number in hex representation.
void si_debug(uint8_t c, uint8_t n);

// Convert a half-byte number (0..15) to a hex character ('0'..'f').
uint8_t si_hex(uint8_t nibble);
