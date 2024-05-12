#include <stdio.h>
#include <string.h>

#include "Arduino.h"
#include "si.h"
#include "stm8.h"
#include "hc12.h"

// For communication with existing HC12 devices the packet size needs to match
// the modem baud rate.
#define PACKET_SIZE HC12_PACKET_SIZE_15KBS

uint8_t radio_buf[PACKET_SIZE + 1];

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
  // The UART isn’t actually used in this firmware, because swimcat is linked in
  // which provides the putchar method and enables respective stdio.h functionality.
  // To send logs to UART instead of swimcat, remove mentions of swimcat_flush in
  // this file and remove the swimcat/swimcat.rel linkage from the Makefile.
  // SERIAL_INIT(9600);

  puts("HC12\r");

  pinMode(HC12_SET, INPUT_PULLUP);
  attachInterrupt(HC12_SET, &on_portB, FALLING | RISING); // B5

  // C4 (IRQ): Low while a radio interrupt is pending.
  attachInterrupt(SI_IRQ, &on_portC, FALLING); // C4

  radio_init(si_config_15kbit);

  // The maximum setting (127) is *very* strong.
  // 16 already gets through multiple concrete walls.
  si_set_tx_power(16);

  // \x18 is needed for HC12 to recognize the packet
  // \x0a Is the length of the string
  // The original HC12 firmware doesn’t seem to like certain values in certain
  // positions, esp not \0 bytes, this is why the string is padded with spaces.
  radio_tx(PACKET_SIZE, "\x18\x0a" "OpenHC12\r\n                                     ");

  // Start variable length RX.
  si_start_rx(0);
}

// Utility function to dump a packet to stdout.
void dump_packet(uint8_t len, uint8_t *data) {
  putchar(':');
  for (int i = 0; i<len; i++) {
    putchar(si_hex(data[i] >> 4));
    putchar(si_hex(data[i] & 0xf));
  }
  putchar('\n');
}

void loop(void) {
  // flush out pending logs, because swimcat doesn’t work in wfi/halt mode
  swimcat_flush();

rx:
  // Receive a packet. This does block until SI_IRQ notifies the MCU that a new
  // packet has arrived.
  uint8_t recvd = radio_rx(PACKET_SIZE, radio_buf);
  if (!recvd) return;

  // The original HC12 format is:
  uint8_t header = radio_buf[0];
  uint8_t pkt_size = radio_buf[1] + 2;
  const char *payload = &radio_buf[2];

  dump_packet(recvd, radio_buf);
  
  if (header != 0x18) {
    puts("Invalid header");
    si_debug('H', header);
    return;
  }

  if (pkt_size < sizeof(radio_buf)) {
    // For nicer output only:
    radio_buf[pkt_size] = 0;
  }
  puts(payload);

  // As an echo server we would now send back the just received packet.
  // But there may be another packet on the way already and since the channel
  // is not full duplex, we would kill both transmissions.

  // A followup packet would normally arrive within ~6.5ms at the standard baud
  // rate, for other rates you need to adjust the time accordingly.
  delay(/*ms=*/8);  
  if (!digitalRead(SI_IRQ)) {
    // The packet handler interrupt triggered. Go straight to the rx.
    goto rx;
  }

  // Send the last packet back.
  // (As the chip is not full duplex, this implies that we may miss additional
  // packets that we might receive during transmission.)
  radio_tx(PACKET_SIZE, radio_buf);

  // To send a shorter packet that the variable-length receiver will recognize:
  // #define HC12_LEN_ADJUST (PACKET_SIZE - 1 - 0x18);
  // uint8_t payload_len = strlen(payload);
  // uint8_t *packet = radio_buf + 1;
  // packet[0] = payload_len + HC12_LEN_ADJUST;
  // radio_tx(payload_len + 1, packet);

  // Restart variable length RX.
  si_start_rx(0);
}
