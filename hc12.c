#include <stdio.h>

#include "Arduino.h"
#include "stm8.h"
#include "si.h"

uint8_t interrupt_state;

void on_port3() {  // IO1 / IRQ
  interrupt_state = PIN3;
  putchar('C');
  if (digitalRead(SI_IRQ) == 0) {
    putchar('I');
    interrupt_state |= STATE_SI_IRQ;
  }
}

void on_port2() {
  putchar('B');
  interrupt_state = PIN2;
}

void setup() {
  SERIAL_INIT(115200);
  puts("HC12\r");

  pinMode(A3, OUTPUT);
  digitalWrite(A3, 1);

  pinMode(D4, OUTPUT);
  digitalWrite(D4, 1);
  delayMicroseconds(11);
  digitalWrite(D4, 0);

  attachInterrupt(SI_IO0, &on_port2, FALLING | RISING);
  attachInterrupt(SI_IRQ, &on_port3, RISING);

  init_radio();

  radio_rx(0x14, radio_buf);
  digitalWrite(A3, 0);
  radio_tx(0x14, "\x18\x08" "Hello!\r\n");
  radio_rx(0x14, radio_buf);
}

void loop() {
  putchar('L');
  wfi();
  putchar('X');
  uint8_t recvd = radio_rx(0x14, radio_buf);
  debug('X', recvd);
  if (recvd) {
    puts(radio_buf);
    radio_tx(0x14, radio_buf);
  }
}
