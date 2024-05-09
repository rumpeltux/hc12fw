// Definitions specific to the HC12 board.

// Pinout
#define HC12_SET B5
#define HC12_TX D5
#define HC12_RX D6
// Pins interfacing with the Si4463 are defined in si.h instead.

// For communication with existing HC12 devices the packet size needs to match
// the modem baud rate.
#define HC12_PACKET_SIZE_5KBS 12
#define HC12_PACKET_SIZE_15KBS 20  // default
#define HC12_PACKET_SIZE_58KBS 33
#define HC12_PACKET_SIZE_236KBS 49
