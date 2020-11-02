#ifndef MCP2515_H
#define MCP2515_J

/* Instructions */
#define MCP_RESET           0xC0    // 1100 0000
#define MCP_READ            0x03    // 0000 0011
#define MCP_READ_RX         0x90    // 1001 0nm0
#define MCP_WRITE           0x02    // 0000 0010
#define MCP_LOAD_TX         0x40    // 0100 0abc
#define MCP_RTS             0x80    // 1000 0nnn
#define MCP_READ_STATUS     0xA0    // 1010 0000
#define MCP_RX_STATUS       0xB0    // 1011 0000
#define MCP_BIT_MODIFY      0x05    // 0000 0101

/* Control Register Addresses */
#define BFPCTRL     0x0C
#define TXRTSCTRL   0x0D
#define CANSTAT     0xE
#define CANCTRL     0xF
#define TEC         0x1C
#define REC         0x1D
#define CNF3        0x28
#define CNF2        0x29
#define CNF1        0x2A
#define CANINTE     0x2B
#define CANINTF     0x2C
#define EFLG        0x2D
#define TXB0CTRL    0x30
#define TXB1CTRL    0x40
#define TXB2CTRL    0x50
#define RXB0CTRL    0x60
#define RXB1CTRL    0x70

/* BFCTRL */
#define B0BFM       0
#define B1BFM       1
#define B0BFE       2
#define B1BFE       3
#define B0BFS       4
#define B1BFS       5

/* TXRTSCTRL */
#define B0RTSM      0
#define B1RTSM      1
#define B2RTSM      2
#define B0RTS       3
#define B1RTS       4
#define B2RTS       5

/* CANSTAT */
#define ICOD0       1
#define ICOD1       2
#define ICOD2       3
#define OPMOD0      5
#define OPMOD1      6
#define OPMOD2      7

/* CANCTRL */
#define CLKPRE0     0
#define CLKPRE1     1
#define CLKEN       2
#define OSM         3
#define ABAT        4
#define REQOP0      5
#define REQOP1      6
#define REQOP2      7

/* CNF3 */
#define PHSEG20     0
#define PHSEG21     1
#define PHSEG22     2
#define WAKFIL      6
#define SOF         7

/* CNF2 */
#define PRSEG0      0
#define PRSEG1      1
#define PRSEG2      2
#define PHSEG10     3
#define PHSEG11     4
#define PHSEG12     5
#define SAM         6
#define BTLMODE     7

/* CNF1 */
#define BRP0        0
#define BRP1        1
#define BRP2        2
#define BRP3        3
#define BRP4        4
#define BRP5        5
#define SJW0        6
#define SJW1        7

/* CANINTE */
#define RX0IE       0
#define RX1IE       1
#define TX0IE       2
#define TX1IE       3
#define TX2IE       4
#define ERRIE       5
#define WAKIE       6
#define MERRE       7

/* CANINTF */
#define RX0IF       0
#define RX1IF       1
#define TX0IF       2
#define TX1IF       3
#define TX2IF       4
#define ERRIF       5
#define WAKIF       6
#define MERRF       7

/* EFLG */
#define EWARN       0
#define RXWAR       1
#define TXWAR       2
#define RXEP        3
#define TXEP        4
#define TXBO        5
#define RX0OVR      6
#define RX1OVR      7

/* TXB(0:2)CTRL */
#define TXP0        0
#define TXP1        1
#define TXREQ       3
#define TXERR       4
#define MLOA        5
#define ABTF        6

/* RXB(0:1)CTRL */
#define FILHIT0     0
#define BUKT1       1
#define BUKT        2
#define RXRTR       3
#define RXM0        5
#define RXM1        6

#endif