#include "Arduino.h"
#include "si.h"
#include <stdio.h>

uint8_t si_dbg = 0;

uint8_t si_hex(uint8_t nibble) {
  if (nibble > 0xf) { return '.'; }
  nibble += '0';
  if (nibble > '9') nibble += 'a' - '0' - 10;
  return nibble;
}

void hexout(uint8_t byte) {
  putchar(si_hex(byte >> 4));
  putchar(si_hex(byte & 0xf));
}

void hexout16(uint16_t data) {
  hexout(data >> 8);
  hexout(data);
}

void si_debug(uint8_t c, uint8_t n) {
  putchar(c); hexout(n); puts("\r");
}

static void spi_tx(uint8_t len, const uint8_t *data) {
  for(uint8_t i=0; i<len; i++) {
    spi_transfer(data[i]);
  }
}

static void spi_rx(uint8_t len, uint8_t *data) {
  for(uint8_t i=0; i<len; i++) {
    data[i] = spi_transfer(0xFF);
  }
}

static void spi_select_tx(uint8_t len, const uint8_t *data) {
  while(digitalRead(SI_IO1) == 0) ;
  digitalWrite(SI_CS, 0);
  spi_tx(len, data);
  digitalWrite(SI_CS, 1);
}

// Returns 0 on timeout while waiting for cts signal.
static uint8_t si_read_cmd_buf(uint8_t len, uint8_t *dest) {
  uint16_t i=0;
  uint8_t ctsVal;
  do {
    digitalWrite(SI_CS, 0);
    spi_transfer(0x44);
    ctsVal = spi_transfer(0xFF);
    if (ctsVal == 0xFF) {
      spi_rx(len, dest);
    }
    digitalWrite(SI_CS, 1);
  } while(++i && ctsVal != 0xFF);
  if (!i) putchar('c');
  return !!i;
}

// HC12 compatible radio params.

// GPIO config:
// GPIO0: TX_STATE (high while in TX)
// GPIO1: CTS (high when command handler is ready to receive the next signal)
// GPIO2/3: used for radio control (2: ANTENNA_1_SW, 3: ANTENNA_2_SW)
// NIRQ: SYNC_WORD_DETECT (high when a Sync Word is detected, and returns low after the packet is received)
// SDO: POR (output goes low during Power-On Reset and goes high upon completion of POR)
static const uint8_t tx_config[] = {0x13, 0x60, 0x48, 0x56, 0x57, 0x5a, 0x4b};
static const uint8_t rx_config[] = {0x13, 0x60, 0x48, 0x57, 0x56, 0x5a, 0x4b};


// Args: Channel=2, Condition, tx_len (16bit_le), num_repeat
static uint8_t si_tx_cmd_buf[6] = {0x31, 2, 0x10, 0, 0, 0};

/*
State Enum:
0 = No change.
1 = Sleep state.
2 = Spi Active state.
3 = Ready state.
4 = Another enumeration for Ready state.
5 = Tune state for TX.
6 = Tune state for RX.
7 = TX state.
8 = RX state.
*/
// Args: Channel=2, Start delayed, len (16bit_le), timeout_state=0, valid_state=3, invalid_state=3
static uint8_t si_rx_cmd_buf[8] = {0x32, 2, 0, 0, 0, 0, 3, 3};
// len = 2
static uint8_t low_power_si_rx_cmd_buf[5] = {0x32, 0x02, 0, 0, 0x02};
// TX_TUNE
static uint8_t low_power_si_tx_cmd_buf[5] = {0x31, 0x02, 0x50, 0, 0x02};

static const uint8_t cmd_get_int_status15[] = {0x20, 0, 0, 0};

#define SET_PROPERTY_L(prop, len, ...) (len + 4), SET_PROPERTY(prop, len, __VA_ARGS__)
#define SET_PROPERTY(prop, len, ...) 0x11, (prop >> 8), len, (prop & 0xff), __VA_ARGS__

static const uint8_t config_init[] = {
  0x07, 0x02, 0x01, 0x00, 0x01, 0xc9, 0xc3, 0x80, // boot RF_POWER_UP
  0x07, 0x13, 0x60, 0x48, 0x57, 0x56, 0x5a, 0x4b, // gpio
  SET_PROPERTY_L(0x0000, 1, 0x48),
  SET_PROPERTY_L(0x0003, 1, 0x40),
  SET_PROPERTY_L(0x0100, 1, 0x00),
  SET_PROPERTY_L(0x0200, 4, 0x03, 0x07, 0x00, 0x00),
  // Preamble-config: [6, 20, 0, 0x50, 0x31, 0, 0, 0, 0]
  // 6 bytes preamble
  // long preamble-timeout: 5 * 15 (=75) nibbles ~= 2.4ms
  // 0x10 = First bit is 1, calculated from the calculator (default)
  // 0x20 = Preamble tx_length register is in bytes
  // 0x01 = Use standard preamble of 1010 (default)
  SET_PROPERTY_L(0x1000, 0x09, 0x06, 0x14, 0x00, 0x50, 0x31, 0x00, 0x00, 0x00, 0x00),
  SET_PROPERTY_L(0x1100, 0x05, 0x21, 0x89, 0x89, 0x00, 0x00),
  SET_PROPERTY_L(0x1200, 1, 0x81),
  SET_PROPERTY_L(0x1206, 1, 0x02),
  SET_PROPERTY_L(0x1208, 3, 0x00, 0x00, 0x00),
  SET_PROPERTY_L(0x120d, 0x0c, 0x00, 0x40, 0x06, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
  SET_PROPERTY_L(0x1219, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
  SET_PROPERTY_L(0x2000, 0x0c, 0x03, 0x00, 0x07, 0x26, 0x25, 0xa0, 0x01, 0xc9, 0xc3, 0x80, 0x00, 0x22),
  SET_PROPERTY_L(0x200c, 1, 0x22),
  SET_PROPERTY_L(0x2018, 0x08, 0x01, 0x00, 0x08, 0x03, 0x80, 0x00, 0x00, 0x30),
  SET_PROPERTY_L(0x2022, 0x09, 0x00, 0x78, 0x04, 0x44, 0x44, 0x04, 0x44, 0x02, 0x00),
  SET_PROPERTY_L(0x202c, 0x07, 0x00, 0x23, 0x8f, 0xff, 0x00, 0xde, 0xa0),
  SET_PROPERTY_L(0x2035, 1, 0xe2),
  SET_PROPERTY_L(0x2038, 0x09, 0x22, 0x0d, 0x0d, 0x00, 0x1a, 0x40, 0x00, 0x00, 0x28),
  SET_PROPERTY_L(0x2042, 0x0b, 0xa4, 0x03, 0xd6, 0x03, 0x01, 0x0a, 0x01, 0x80, 0xff, 0x0c, 0x00),
  SET_PROPERTY_L(0x204e, 1, 0x40),
  SET_PROPERTY_L(0x2051, 1, 0x0a),
  SET_PROPERTY_L(0x2100, 0x0c, 0x5b, 0x47, 0x0f, 0xc0, 0x6d, 0x25, 0xf4, 0xdb, 0xd6, 0xdf, 0xec, 0xf7),
  SET_PROPERTY_L(0x210c, 0x0c, 0xfe, 0x01, 0x15, 0xf0, 0xff, 0x03, 0x5b, 0x47, 0x0f, 0xc0, 0x6d, 0x25),
  SET_PROPERTY_L(0x2118, 0x0c, 0xf4, 0xdb, 0xd6, 0xdf, 0xec, 0xf7, 0xfe, 0x01, 0x15, 0xf0, 0xff, 0x03),
  SET_PROPERTY_L(0x2200, 4, 0x08, 0x7f, 0x00, 0x5d),
  SET_PROPERTY_L(0x2300, 0x07, 0x01, 0x05, 0x0b, 0x05, 0x02, 0x00, 0x03),
  SET_PROPERTY_L(0x3000, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00),
  SET_PROPERTY_L(0x4000, 0x08, 0x38, 0x0d, 0xdd, 0xdd, 0x36, 0x9d, 0x20, 0xfe),
  0x04, 0x20, 0, 0, 0,
  0};

static const uint8_t config_fu3[] = {
  SET_PROPERTY(0x2000, 12, 0x03, 0x00, 0x07, 0x00, 0xea, 0x60, 0x04, 0x2d, 0xc6, 0xc0, 0x00, 0x04),
  SET_PROPERTY(0x200c, 1, 0x19),
  SET_PROPERTY(0x2018, 8, 0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0x20, 0x10),
  SET_PROPERTY(0x2022, 9, 0x00, 0xa7, 0x03, 0x12, 0x6f, 0x01, 0x88, 0x02, 0xc2),
  SET_PROPERTY(0x202c, 7, 0x04, 0x36, 0x80, 0x2c, 0x07, 0xe9, 0x80),
  SET_PROPERTY(0x2038, 9, 0x11, 0x25, 0x25, 0x00, 0x1a, 0x80, 0x00, 0x00, 0x29),
  SET_PROPERTY(0x2042, 11, 0xa4, 0x02, 0xd6, 0x83, 0x01, 0x44, 0x01, 0x80, 0xff, 0x0c, 0x00),
  SET_PROPERTY(0x2100, 12, 0xcc, 0xa1, 0x30, 0xa0, 0x21, 0xd1, 0xb9, 0xc9, 0xea, 0x05, 0x12, 0x11),
  SET_PROPERTY(0x210c, 12, 0x0a, 0x04, 0x15, 0xfc, 0x03, 0x00, 0xcc, 0xa1, 0x30, 0xa0, 0x21, 0xd1),
  SET_PROPERTY(0x2118, 12, 0xb9, 0xc9, 0xea, 0x05, 0x12, 0x11, 0x0a, 0x04, 0x15, 0xfc, 0x03, 0x00),
  SET_PROPERTY(0x2200, 4, 0x08, 0x7f, 0x00, 0x3d),
  SET_PROPERTY(0x2300, 7, 0x2c, 0x0e, 0x0b, 0x04, 0x0c, 0x73, 0x03),
  SET_PROPERTY(0x2201, 1, 0x7f),
  0
};

static const uint8_t config_fu2[] = {
  SET_PROPERTY(0x0001, 1, 0x01),
  // WUT: 0, 15, 92, 32, 13, 1
  // WUT_M: 3932 (4 * 3932 / 32.768 ~= .5s)
  // WUT_R: 0x20 Go to Sleep state after WUT, R=0
  // WUT_LDC: 0x0d (13) -> 4*13/ 32.768 (1.5ms)
  SET_PROPERTY(0x0004, 6, 0x00, 0x0f, 0x5c, 0x20, 0x0d, 0x01),
  SET_PROPERTY(0x2201, 1, 0x7f),
  0
};

const uint8_t power_consts[]  = {4, 6, 9, 13, 18, 26, 40, 127};

void si_set_tx_power(uint8_t power) {
  uint8_t cmd[] = {SET_PROPERTY(0x2201, 1, power)};
  spi_select_tx(sizeof(cmd), cmd);
}

void si_set_channel(uint8_t channel) {
  channel++;
  si_tx_cmd_buf[1] = channel;
  si_rx_cmd_buf[1] = channel;
}

// Returns the chip part number, e.g. 0x4463
uint16_t si_get_chip(void) {
  uint8_t cmd[] = {0x01};  // GET_CHIP_INFO
  spi_select_tx(sizeof(cmd), cmd);
  uint8_t chip_info[8];
  uint8_t chip_info_success = si_read_cmd_buf(8, chip_info);
  if (!chip_info_success) return 0;
  return chip_info[1] << 8 | chip_info[2];
}

uint8_t radio_init() {
  digitalWrite(SI_CS, 1);
  pinMode(SI_CS, OUTPUT);
  
  pinMode(SI_IRQ, INPUT);
  pinMode(SI_IO0, INPUT);
  pinMode(SI_IO1, INPUT);

  spi_begin();
  // Set speed to 8MHz (Si4463 is max 10MHz)
  SPI_CR1 = 0x44;

  // Wait until radio chip is ready to respond.
  while(digitalRead(SI_IO1) == 0) yield();

  // Sanity check to confirm that peripheral communication
  // and the radio chip are working.
  int64_t chip = si_get_chip();
  if (chip != 0x4463) {
    // unexpected / unsupport chip (or read failure when 0)
    puts("CHIP:");
    hexout16(chip);
    return 0;
  }

  // Send initial radio params.
  const uint8_t *cmd_p = config_init;
  uint8_t len;
  while((len = *cmd_p) != 0) {
    spi_select_tx(len, ++cmd_p);
    cmd_p += len;
  }

  // Send mode specific radio params (for now only FU3 is know to be working)
  cmd_p = config_fu3;
  do {
    len = cmd_p[2] + 4;
    spi_select_tx(len, cmd_p);
    cmd_p += len;
  } while (*cmd_p != 0);

  // Reasonable default params (compatible with HC12’s AT+DEFAULT)
  si_set_channel(1);
  return 1;
}

// 1: sleep
void si_change_state(uint8_t state) {
  digitalWrite(SI_CS, 0);
  spi_transfer(0x34); // change state
  spi_transfer(state);
  digitalWrite(SI_CS, 1);
}

void wait_radio_tx_done() {
  uint16_t i=0;
  while(digitalRead(SI_IO0) == 0 && ++i) ;
  uint8_t fail=i == 0;
  while(digitalRead(SI_IO0) != 0 && ++i) ;
  if (!i) fail |= 2;
  if (fail) si_debug('T', fail);
}

void radio_rx_mode() {
  spi_select_tx(sizeof(rx_config), rx_config);
}

void radio_tx(uint8_t len, const uint8_t *data) {
  spi_select_tx(sizeof(tx_config), tx_config);

  // Fill TX Fifo with data.  
  digitalWrite(SI_CS, 0);
  spi_transfer(0x66); // TX_FIFO
  spi_tx(len, data);
  digitalWrite(SI_CS, 1);

  // Issue TX command
  si_tx_cmd_buf[4] = len;
  spi_select_tx(sizeof(si_tx_cmd_buf), si_tx_cmd_buf);

  wait_radio_tx_done();
  radio_rx_mode();
}

static const uint8_t fifo_info0[] = {0x15, 0};
static const uint8_t fifo_info3[] = {0x15, 3};

static void si_read_rx_fifo_info(uint8_t len, uint8_t *dest) {
  digitalWrite(SI_CS, 0);
  spi_transfer(0x77);  // READ_RX_FIFO
  spi_rx(len, dest);
  digitalWrite(SI_CS, 1);
}

static const uint8_t request_device_state[] = {0x33};

static void si_dump_interrupt_state(uint8_t *interrupts) {
  if (si_dbg) {
    putchar('I'); // interrupt state
    for (uint8_t i=0;i<8; i++) hexout(interrupts[i]);
  }
}

static void si_ensure_rx(void) {
   uint8_t device_state[3];
   // Requests the current state of the device and lists pending TX and RX requests
   spi_select_tx(1, request_device_state);
  uint8_t device_state_success = si_read_cmd_buf(2, device_state);
  // Old AN625: state, channel
  if (device_state_success && (device_state[0] & 0xF) != 8) { // not in RX state
     // Start next RX.
    spi_select_tx(sizeof(si_rx_cmd_buf), si_rx_cmd_buf);
   }
}

// len is sizeof dest and must be >= 8 since we reuse the buffer
uint8_t radio_rx(uint8_t len, uint8_t *dest) {
  uint8_t ready_bytes = 0;
  si_rx_cmd_buf[4] = len;
  while (digitalRead(SI_IRQ) != 0)
    ;

  // get payload length
  spi_select_tx(2, fifo_info0);
  uint8_t fifo_info_success = si_read_cmd_buf(2, dest);
  if (!fifo_info_success)
    goto rx_exit;

  ready_bytes = dest[0];
  if (!ready_bytes)
    goto rx_exit;

  // Check pending interrupts to confirm that data is available.
  spi_select_tx(4, cmd_get_int_status15);
  uint8_t *interrupts = dest;
  uint8_t int_success = si_read_cmd_buf(8, interrupts);
  if (!int_success) {
    ready_bytes = 0;
    goto rx_exit;
  }
  si_dump_interrupt_state(interrupts);

  if ((interrupts[2] & 0x10) != 0) { // RX pending
    si_read_rx_fifo_info(ready_bytes, dest);
  } else {
    if ((interrupts[2] & 0x8) != 0) { // CRC error
      // reset RX & TX fifos
      spi_select_tx(2, fifo_info3);
    }
    ready_bytes = 0;
  }

rx_exit:
  si_ensure_rx();
  return ready_bytes;
}

void radio_wakeup() {
  // TODO (may not be necessary)
}

void radio_halt() {
  // TODO: disable 32K osc
  si_change_state(1);  // go to sleep
}
