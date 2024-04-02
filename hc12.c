#include <stdio.h>

#include "Arduino.h"
#include "stm8.h"
#include "si.h"

extern uint8_t interrupt_state;
uint8_t radio_buf2[0x40];

extern void swimcat_flush();

void on_portC() {  // IO1 / IRQ
  //interrupt_state = PIN3;
  putchar('C');
  //debug('C', PC_IDR & 0x10);
  if (digitalRead(SI_IRQ) == 0) {
    putchar('I');
    si_interrupt_handler();
  }
}

void on_port2() {
  putchar('B');
  //interrupt_state = PIN2;
}

void setup() {
  puts("HC12\r");

  pinMode(A3, OUTPUT);
  digitalWrite(A3, 1);

  pinMode(D4, OUTPUT);
  digitalWrite(D4, 1);
  delayMicroseconds(11);
  digitalWrite(D4, 0);
  pinMode(B5, INPUT_PULLUP);

  attachInterrupt(SI_IO0, &on_port2, FALLING | RISING); // B4
  attachInterrupt(B5, &on_port2, FALLING | RISING); // B5
  attachInterrupt(SI_IRQ, &on_portC, FALLING); // C4

  init_radio();
  yield();
  fu2_start_rx(/*long_mode=*/ 0);

  // radio_rx(0x14, radio_buf);
  // digitalWrite(A3, 0);
  // radio_tx(0x14, "\x18\x08" "Hello!\r\n");
  // radio_rx(0x14, radio_buf);
}

void handle_flare(uint16_t flare_idx) {
  if (!flare_idx) { putchar('l'); return; }
  putchar('L');
  hexout16(flare_idx);
  debug('P', si_latched_rssi); // power

  flare_idx = 866 - flare_idx;
  // flare_idx is now a countdown timer in units of 560us

  if (flare_idx > 50) { 
    putchar('W');
    swimcat_flush();
    flare_idx -= 40;
    delay(flare_idx / 2);
    return;
  }
  // 35 is 0.02s which is enough time for swimcat to catch up.
  // double it to be sure. (only need 14 from experiments)
  //if (flare_idx > 70) { puts("F"); swimcat_flush(); putchar('F'); return; }
  putchar('D'); hexout16(flare_idx);

  if (flare_idx > 0) {
    //flare_idx-=5;
  }
  //flare_idx += 3;
  putchar('.');
  delay(flare_idx / 2);
  delayMicroseconds(flare_idx * 60);
  putchar('.');
  fu2_start_rx(/*long=*/1);

  // TODO: set timeout
  uint32_t x = 0x100000;
  while(interrupt_state == 0 && --x != 0) yield();
  hexout(x >> 16); hexout16(x);
  if (!x) {
    debug('S', si_get_state());
    uint8_t interrupts[8];
    si_read_interrupt_status2(interrupts);
    for (uint8_t i=0; i<8; i++) { putchar(' '); hexout(interrupts[i]); }
  }
  if (interrupt_state & INTERRUPT_STATE_RX_READY) {
    uint8_t recvd = si_read_fifo(0x3F, radio_buf);
    debug('X', recvd); // received bytes
    debug('P', si_latched_rssi);
    puts(radio_buf);
    for (uint8_t i = 0; i < 0x10; i++) {
      hexout(radio_buf[i]); putchar(' ');
    }
    putchar('\n');
  } else if (interrupt_state & INTERRUPT_STATE_RX_ERROR) {
    puts("CRC!");
  } else {
    debug('W', interrupt_state);
  }
  puts("done");
}

void loop() {
  putchar('L'); //swimcat_flush(); // going to wfi
  while(interrupt_state == 0) {
    //delay(10);
    /*debug('S', si_get_state());
    uint8_t interrupts[8];
    si_read_interrupt_status2(interrupts);
    for (uint8_t i=0; i<8; i++) { putchar(' '); hexout(interrupts[i]); }*/
    yield();
  }
  putchar('i');
  if (interrupt_state & INTERRUPT_STATE_RX_READY) {
    uint16_t flare_idx;
    uint8_t recvd = si_read_fifo(2, &flare_idx);
    if (recvd == 2) {
      si_stop_rx();
      handle_flare(flare_idx);
    } else {
      puts("WTF!");
    }
  } else if (interrupt_state & INTERRUPT_STATE_RX_ERROR) {
    debug('P', si_latched_rssi);
    puts("CRC");
    uint8_t recvd2 = si_read_fifo(4, radio_buf);
    debug('x', recvd2); // received bytes
    hexout16(*(uint16_t *) radio_buf);
    hexout16(*(uint16_t *) (radio_buf + 2));
    putchar('\n');
  } else {
    debug('U', interrupt_state);
    puts("<-UNKNOWN INT");
    interrupt_state = 0;
  }
  fu2_start_rx(/*long_mode=*/ 0);
}
