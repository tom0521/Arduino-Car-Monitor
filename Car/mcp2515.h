#ifndef MCP2515_H
#define MCP2515_J

/* Instructions */
#define RESET       0xC0    // 1100 0000
#define READ        0x03    // 0000 0011
#define WRITE       0x02    // 0000 0010
#define STATUS      0xA0    // 1010 0000
#define RX_STATUS   0xB0    // 1011 0000
#define BIT_MODIFY  0x05    // 0000 0101

/* CNF3 Register bits */
#define PHSEG21     1

#endif