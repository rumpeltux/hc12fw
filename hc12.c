#include <stdio.h>
#include <string.h>

#include "Arduino.h"
#include "si.h"
#include "stm8.h"

uint8_t radio_buf[21];

#define HC12_SET B5

extern void swimcat_flush(void);

void on_portC(void) {  // IO1 / IRQ
  if (digitalRead(SI_IRQ) == 0) {
    si_notify_nirq();
  }
}

void on_portB(void) {
  puts("SET");
}

void setup(void) {
  SERIAL_INIT(9600);
  puts("HC12\r");

  pinMode(HC12_SET, INPUT_PULLUP);
  attachInterrupt(HC12_SET, &on_portB, FALLING | RISING); // B5

  // C4 (IRQ): Low while a radio interrupt is pending.
  attachInterrupt(SI_IRQ, &on_portC, FALLING); // C4

  radio_init();

  // The maximum setting (127) is very strong.
  // 16 already gets through multiple concrete walls.
  si_set_tx_power(16);

  // 20 is the packet length.
  // Other lengths are not supported in FU3 mode!
  // \x18 is needed for HC12 to recognize the packet
  // \x0c Is the length of the string (max: 17 (0x11))
  radio_tx(20, "\x18\x0c" "HC12 ready\r\n");

  // Start variable length RX
  radio_start_rx(0);

#ifdef RANGE_TEST
  // Power and range test (disabled by default).
  strcpy(radio_buf, "\x18\x10" "TX power: 0x00\r\n");
  for (uint8_t i=0x7f; i>0; i-=2) {
    si_set_tx_power(i);
    radio_buf[14] = si_hex(i >> 4);
    radio_buf[15] = si_hex(i & 0xf);
    radio_tx(20, radio_buf);
  }
  si_set_tx_power(0x10);
#endif
}

void loop(void) {
  // flush out pending logs, because swimcat doesn’t work while were in sleep mode
  swimcat_flush();

rx:
  // Wait until an interrupt triggered that is of our concern.
  uint8_t recvd = radio_rx(20, radio_buf);
  if (!recvd) return;

  uint8_t header = radio_buf[0];
  uint8_t pkt_size = radio_buf[1] + 2;
  const char *payload = &radio_buf[2];
  if (header != 0x18) {
    puts("Invalid header");
    si_debug('H', header);
    return;
  }
  if (pkt_size < sizeof(radio_buf))
    radio_buf[pkt_size] = 0;
  puts(payload);

  // Check if there's another packet on the way.
  // It would normally arrive within ~6.5ms
  delay(8);
  if (!digitalRead(SI_IRQ))
    goto rx;

  // Send the last packet back.
  // (As the chip is not full duplex, this implies that we may miss additional
  // packets that we might receive during transmission.)
  radio_tx(0x14, radio_buf);

  // To send a shorter packet that the variable-length receiver will recognize:
  // uint8_t len = strlen(payload) + 2;
  // radio_buf[0] = len + 4;
  // radio_tx(len, radio_buf);

  // Restart variable length RX.
  radio_start_rx(0);
}
