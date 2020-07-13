#include "Arduino.h"
#include "si.h"
#include <stdio.h>

uint8_t radio_buf[0x40];
uint8_t dbg = 0;

static void _hexout(uint8_t nibble) {
  nibble += '0';
  if (nibble > '9') nibble += 39;
  putchar(nibble);
}

void hexout(uint8_t byte) {
  _hexout(byte >> 4);
  _hexout(byte & 0xf);
}

void hexout16(uint16_t data) {
  hexout(data >> 8);
  hexout(data);
}

void spi_tx(uint8_t len, const uint8_t *data) {
  if (dbg) {
    putchar('S'); hexout(len); putchar(':');
    for(uint8_t i=0; i<len; i++) {
      hexout(data[i]);
    }
    puts("\r");
  }
  for(uint8_t i=0; i<len; i++) {
    spi_transfer(data[i]);
  }
}

void spi_rx(uint8_t len, uint8_t *data) {
  for(uint8_t i=0; i<len; i++) {
    data[i] = spi_transfer(0xFF);
  }
}

void spi_select_tx(uint8_t len, const uint8_t *data) {
  while(digitalRead(SI_IO1) == 0) ;
  digitalWrite(SI_CS, 0);
  spi_tx(len, data);
  digitalWrite(SI_CS, 1);
}

void debug(uint8_t c, uint8_t n) {
  putchar(c); hexout(n); puts("\r");
}

static const uint8_t tx_config[] = {0x13, 0x60, 0x48, 0x56, 0x57, 0x5a, 0x4b};
static const uint8_t rx_config[] = {0x13, 0x60, 0x48, 0x57, 0x56, 0x5a, 0x4b};
static uint8_t si_tx_cmd_buf[6] = {0x31, 0xE6, 0x10, 0, 0, 0};
static uint8_t si_rx_cmd_buf[8] = {0x32, 0xE6, 0, 0, 0, 0, 1, 3};

static const uint8_t cmd_get_int_status15[] = {0x20, 0, 0, 0};
static const uint8_t radio_init[] = {
  0x07, 0x02, 0x01, 0x00, 0x01, 0xc9, 0xc3, 0x80, 0x07, 0x13, 0x60, 0x48,
  0x57, 0x56, 0x5a, 0x4b, 0x05, 0x11, 0x00, 0x01, 0x00, 0x48, 0x05, 0x11,
  0x00, 0x01, 0x03, 0x40, 0x05, 0x11, 0x01, 0x01, 0x00, 0x00, 0x08, 0x11,
  0x02, 0x04, 0x00, 0x03, 0x07, 0x00, 0x00, 0x0d, 0x11, 0x10, 0x09, 0x00,
  0x06, 0x14, 0x00, 0x50, 0x31, 0x00, 0x00, 0x00, 0x00, 0x09, 0x11, 0x11,
  0x05, 0x00, 0x21, 0x89, 0x89, 0x00, 0x00, 0x05, 0x11, 0x12, 0x01, 0x00,
  0x81, 0x05, 0x11, 0x12, 0x01, 0x06, 0x02, 0x07, 0x11, 0x12, 0x03, 0x08,
  0x00, 0x00, 0x00, 0x10, 0x11, 0x12, 0x0c, 0x0d, 0x00, 0x40, 0x06, 0xaa,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x11, 0x12, 0x08,
  0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x20,
  0x0c, 0x00, 0x03, 0x00, 0x07, 0x26, 0x25, 0xa0, 0x01, 0xc9, 0xc3, 0x80,
  0x00, 0x22, 0x05, 0x11, 0x20, 0x01, 0x0c, 0x22, 0x0c, 0x11, 0x20, 0x08,
  0x18, 0x01, 0x00, 0x08, 0x03, 0x80, 0x00, 0x00, 0x30, 0x0d, 0x11, 0x20,
  0x09, 0x22, 0x00, 0x78, 0x04, 0x44, 0x44, 0x04, 0x44, 0x02, 0x00, 0x0b,
  0x11, 0x20, 0x07, 0x2c, 0x00, 0x23, 0x8f, 0xff, 0x00, 0xde, 0xa0, 0x05,
  0x11, 0x20, 0x01, 0x35, 0xe2, 0x0d, 0x11, 0x20, 0x09, 0x38, 0x22, 0x0d,
  0x0d, 0x00, 0x1a, 0x40, 0x00, 0x00, 0x28, 0x0f, 0x11, 0x20, 0x0b, 0x42,
  0xa4, 0x03, 0xd6, 0x03, 0x01, 0x0a, 0x01, 0x80, 0xff, 0x0c, 0x00, 0x05,
  0x11, 0x20, 0x01, 0x4e, 0x40, 0x05, 0x11, 0x20, 0x01, 0x51, 0x0a, 0x10,
  0x11, 0x21, 0x0c, 0x00, 0x5b, 0x47, 0x0f, 0xc0, 0x6d, 0x25, 0xf4, 0xdb,
  0xd6, 0xdf, 0xec, 0xf7, 0x10, 0x11, 0x21, 0x0c, 0x0c, 0xfe, 0x01, 0x15,
  0xf0, 0xff, 0x03, 0x5b, 0x47, 0x0f, 0xc0, 0x6d, 0x25, 0x10, 0x11, 0x21,
  0x0c, 0x18, 0xf4, 0xdb, 0xd6, 0xdf, 0xec, 0xf7, 0xfe, 0x01, 0x15, 0xf0,
  0xff, 0x03, 0x08, 0x11, 0x22, 0x04, 0x00, 0x08, 0x7f, 0x00, 0x5d, 0x0b,
  0x11, 0x23, 0x07, 0x00, 0x01, 0x05, 0x0b, 0x05, 0x02, 0x00, 0x03, 0x10,
  0x11, 0x30, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0c, 0x11, 0x40, 0x08, 0x00, 0x38, 0x0d, 0xdd,
  0xdd, 0x36, 0x9d, 0x20, 0xfe, 4, 0x20, 0, 0, 0, 0};
  
static const uint8_t config14[] = {
  0x11, 0x20, 0x0c, 0x00, 0x03, 0x00, 0x07, 0x00, 0xea, 0x60, 0x04, 0x2d,
  0xc6, 0xc0, 0x00, 0x04, 0x11, 0x20, 0x01, 0x0c, 0x19, 0x11, 0x20, 0x08,
  0x18, 0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0x20, 0x10, 0x11, 0x20, 0x09,
  0x22, 0x00, 0xa7, 0x03, 0x12, 0x6f, 0x01, 0x88, 0x02, 0xc2, 0x11, 0x20,
  0x07, 0x2c, 0x04, 0x36, 0x80, 0x2c, 0x07, 0xe9, 0x80, 0x11, 0x20, 0x09,
  0x38, 0x11, 0x25, 0x25, 0x00, 0x1a, 0x80, 0x00, 0x00, 0x29, 0x11, 0x20,
  0x0b, 0x42, 0xa4, 0x02, 0xd6, 0x83, 0x01, 0x44, 0x01, 0x80, 0xff, 0x0c,
  0x00, 0x11, 0x21, 0x0c, 0x00, 0xcc, 0xa1, 0x30, 0xa0, 0x21, 0xd1, 0xb9,
  0xc9, 0xea, 0x05, 0x12, 0x11, 0x11, 0x21, 0x0c, 0x0c, 0x0a, 0x04, 0x15,
  0xfc, 0x03, 0x00, 0xcc, 0xa1, 0x30, 0xa0, 0x21, 0xd1, 0x11, 0x21, 0x0c,
  0x18, 0xb9, 0xc9, 0xea, 0x05, 0x12, 0x11, 0x0a, 0x04, 0x15, 0xfc, 0x03,
  0x00, 0x11, 0x22, 0x04, 0x00, 0x08, 0x7f, 0x00, 0x3d, 0x11, 0x23, 0x07,
  0x00, 0x2c, 0x0e, 0x0b, 0x04, 0x0c, 0x73, 0x03, 0};

const uint8_t power_consts[]  = {4, 6, 9, 13, 18, 26, 40, 127};

void si_set_tx_power(uint8_t power) {
  uint8_t cmd[] = {0x11, 0x22, 1, 1, power};
  spi_select_tx(5, cmd);
}

void si_set_channel(uint8_t channel) {
  si_tx_cmd_buf[1] = channel;
  si_rx_cmd_buf[1] = channel;
}

void init_radio() {
  spi_begin();
  putchar('w');
  while(digitalRead(SI_IO1) == 0) ;
  putchar('!');

  const uint8_t *cmd_p = radio_init;
  uint8_t len;
  while((len = *cmd_p) != 0) {
    debug('c', len);
    spi_select_tx(len, ++cmd_p);
    cmd_p += len;
  }
  
  cmd_p = config14;
  do {
    len = cmd_p[2] + 4;
    debug('C', len);
    spi_select_tx(len, cmd_p);
    cmd_p += len;
  } while (*cmd_p != 0);
  
  si_set_tx_power(0x7F);  // full power
  si_set_channel(2);
  puts("RX\r");
}

void radio_tx(uint8_t len, const uint8_t *data) {
  dbg = 0;
  putchar('T');
  spi_select_tx(sizeof(tx_config), tx_config);

  digitalWrite(SI_CS, 0);
  spi_transfer(0x66); // TX_FIFO
  spi_tx(len, data);
  digitalWrite(SI_CS, 1);
  putchar('1');

  si_tx_cmd_buf[4] = len;
  spi_select_tx(sizeof(si_tx_cmd_buf), si_tx_cmd_buf);
  putchar('2');

  uint8_t i=0;
  while(digitalRead(SI_IO0) == 0 && ++i) hexout(i);
  putchar(',');
  while(digitalRead(SI_IO0) != 0 && ++i) hexout(i);
  putchar('!');
  
  dbg = 1;
  spi_select_tx(sizeof(rx_config), rx_config);
  puts("\r");
}

static const uint8_t fifo_info0[] = {0x15, 0};
static const uint8_t fifo_info3[] = {0x15, 3};

static uint8_t si_read_cmd_buf(uint8_t len, uint8_t *dest) {
  uint16_t i=1;
  while(++i) {
    digitalWrite(SI_CS, 0);
    spi_transfer(0x44);
    uint8_t ctsVal = spi_transfer(0xFF);
    if (ctsVal == 0xFF) {
      if (len) {
        spi_rx(len, dest);
      }
      digitalWrite(SI_CS, 1);
      break;
    }
    digitalWrite(SI_CS, 1);
  }
  return !!i;
}

static void si_read_rx_fifo_info(uint8_t len, uint8_t *dest) {
  digitalWrite(SI_CS, 0);
  spi_transfer(0x77);  // READ_RX_FIFO
  spi_rx(len, dest);
  digitalWrite(SI_CS, 1);
}

static const uint8_t request_device_state[] = {0x33};

uint8_t radio_rx(uint8_t arg, uint8_t *dest) {
  si_rx_cmd_buf[4] = arg;
  putchar('W');
  while(digitalRead(SI_IRQ) != 0) ;
  putchar('!');
  dbg = 1;
  spi_select_tx(2, fifo_info0);
  uint8_t r[2];
  uint8_t res0 = si_read_cmd_buf(2, r);
  
  uint8_t ready = r[0];
  //debug('R', ready);
  if (res0 && ready) {
    spi_select_tx(4, cmd_get_int_status15);
    si_read_cmd_buf(7, dest + 1);
    debug('P', dest[3]);
    if (dest[3] & 0x10 != 0) // ph_pend
    {
      si_read_rx_fifo_info(ready, dest);
    } else {
      if (dest[3] & 0x8 != 0) {
        spi_select_tx(2, fifo_info3);
      }
      ready = 0;
    }
  }

  uint8_t device_state[3];
  dbg = 1;
  spi_select_tx(1, request_device_state);
  uint8_t res1 = si_read_cmd_buf(3, device_state);
  debug('R', r[0]);
  debug('R', r[1]);
  if(res0) putchar('0');
  if(res1) {
    debug('S', device_state[0]);
  }
  if (res1 && (device_state[0] & 0xF) != 8) {
    spi_select_tx(sizeof(si_rx_cmd_buf), si_rx_cmd_buf);
  }
  putchar('!');
  puts("\r");
  return ready;
}

const uint8_t get_modem_status[] = {0x22, 0};

#if 0
void radio_rx2() {
  /*uint8_t modem_status[8];
  spi_select_tx(sizeof(get_modem_status), get_modem_status);
  si_read_cmd_buf(8, modem_status);
  debug('X', modem_status[4]);*/
  while(digitalRead(SI_IRQ) != 0) ;
  //putchar('!');
  spi_select_tx(2, fifo_info0);
  uint8_t r[2];
  uint8_t res0 = si_read_cmd_buf(2, r);
  uint8_t ready = r[0];
  if (ready) {
    spi_select_tx(2, cmd_get_int_status15);
    si_read_cmd_buf(7, dest + 1);
}
#endif
