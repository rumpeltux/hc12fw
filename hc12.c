#include <stdio.h>
#include <string.h>

#include "Arduino.h"
#include "stm8.h"
#include "si.h"


uint8_t interrupt_state=0;
uint8_t radio_buf[21];

#define HC12_SET B5

extern void swimcat_flush();

void on_portC() {  // IO1 / IRQ
  interrupt_state = PIN3;
  if (digitalRead(SI_IRQ) == 0) {
    interrupt_state |= STATE_SI_IRQ;
  }
}

void on_portB() {
  interrupt_state = PIN2;
}

void setup() {
  SERIAL_INIT(9600);
  puts("HC12\r");

  pinMode(HC12_SET, INPUT_PULLUP);

  attachInterrupt(SI_IO0, &on_portB, FALLING | RISING); // B4
  attachInterrupt(HC12_SET, &on_portB, FALLING | RISING); // B5
  attachInterrupt(SI_IRQ, &on_portC, RISING); // C4

  radio_init();

  // The maximum setting (127) is very strong.
  // 16 already gets through multiple concrete walls.
  si_set_tx_power(16);

  // 20 is the packet length.
  // Other lengths are not supported in FU3 mode!
  // \x18 is needed for HC12 to recognize the packet
  // \x0c Is the length of the string (max: 17 (0x11))
  radio_tx(20, "\x18\x0c" "HC12 ready\r\n");

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

void loop() {
  // flush out pending logs, because swimcat doesn’t work while were in sleep mode
  swimcat_flush();

  // Wait until an interrupt triggered that is of our concern.
  disableInterrupts();
  while(interrupt_state == 0) { wfi(); yield(); }
  interrupt_state = 0;
  enableInterrupts();

  // Check if a packet arrived.
  uint8_t recvd = radio_rx(0x14, radio_buf);
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

  // Send the packet back.
  // (As the chip is not full duplex, this implies that we may miss additional
  // packets that we might receive during transmission.)
  radio_tx(0x14, radio_buf);
}
