#include <stdio.h>

#include "Arduino.h"
#include "stm8.h"
#include "si.h"

uint8_t interrupt_state=0;

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
  puts("HC12\r");

  pinMode(A3, OUTPUT);
  digitalWrite(A3, 1);

  pinMode(D4, OUTPUT);
  digitalWrite(D4, 1);
  delayMicroseconds(11);
  digitalWrite(D4, 0);
  pinMode(B5, INPUT_PULLUP);

  attachInterrupt(SI_IO0, &on_port2, FALLING | RISING); // B4
  attachInterrupt(B5, &on_port2, FALLING | RISING); // B4
  attachInterrupt(SI_IRQ, &on_port3, RISING); // C4

  init_radio();

  // radio_rx(0x14, radio_buf);
  // digitalWrite(A3, 0);
  // radio_tx(0x14, "\x18\x08" "Hello!\r\n");
  // radio_rx(0x14, radio_buf);
}

extern void swimcat_flush();
uint8_t radio_buf2[0x40];
void loop() {
  //putchar('L'); //swimcat_flush(); // going to wfi
  while(interrupt_state == 0) handle_events();
  uint16_t flare_idx;
  uint8_t recvd = fu2_radio_rx(2, &flare_idx);
  
  if (recvd) {
    putchar('L');
    hexout16(flare_idx);
    debug('P', si_latched_rssi);
    flare_idx = 866 - flare_idx;
    // flare_idx is now a countdown timer in units of 560us
    
    // 35 is 0.02s which is enough time for swimcat to catch up.
    // double it to be sure. (only need 14 from experiments)
    if (flare_idx > 600) { delayMicroseconds(56000); return; }
    if (flare_idx > 70) { puts("F"); swimcat_flush(); putchar('F'); return; }
    putchar('D'); hexout16(flare_idx);


    if (flare_idx > 0) {
      //flare_idx-=2;
    }
    flare_idx += 3;
    recvd = 0; //fu2_radio_rx_long(0x3F, radio_buf2);
    putchar('.');
    delay(flare_idx / 2);
    delayMicroseconds(flare_idx * 60);
    putchar('.');
    
    uint8_t loop = 0xff;
    while(!recvd && --loop) {
      recvd = fu2_radio_rx_long(0x3F, radio_buf);
      delay(10); putchar('-');
    }
    debug('X', recvd); // received bytes
    debug('P', si_latched_rssi);
    puts(radio_buf);
    for (uint8_t i = 0; i < 0x10; i++) {
      hexout(radio_buf[i]); putchar(' ');
    }
    putchar('\n');
    puts(radio_buf2);
    for (uint8_t i = 0; i < 0x10; i++) {
      hexout(radio_buf2[i]); putchar(' ');
    }
    putchar('\n');
  }

  /*
  interrupt_state = 0;
  putchar('X'); // wakeup from interrupt
  uint8_t recvd = radio_rx(0x14, radio_buf);
  debug('X', recvd); // received bytes
  if (recvd) {
    puts(radio_buf);
    radio_tx(0x14, radio_buf);
  }*/
}
