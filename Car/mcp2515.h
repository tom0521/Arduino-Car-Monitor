#ifndef MCP2515_H
#define MCP2515_J

/* Instructions */
#define MCP_RESET           0xC0    // Reinitialize the internal registers and set the configuration mode
#define MCP_READ            0x03    // Read instruction
#define MCP_READ_RX         0x90    // Quicky address and receive buffer for reading
#define MCP_WRITE           0x02    // Write instruction
#define MCP_LOAD_TX         0x40    // Quicky write to a transmit buffer
#define MCP_RTS             0x80    // Initiate message transmission for one or more of the transmit buffers
#define MCP_READ_STATUS     0xA0    // Allows single instruction action to some of the often used status bits
#define MCP_RX_STATUS       0xB0    // Quicky determine which filter matched the message and maessage type
#define MCP_BIT_MODIFY      0x05    // Setting or clearing individual bits in specific status and control registers

/* Control Register Addresses */
#define BFPCTRL     0x0C        // RXnBF Pin Control and Status Register
#define TXRTSCTRL   0x0D        // TXnRTS Pin Control and Status Register
#define CANSTAT     0xE         // CAN Status Register
#define CANCTRL     0xF         // CAM Control Register
#define TEC         0x1C        // Transmit Error Counter Register
#define REC         0x1D        // Receive Error Counter Register
#define CNF3        0x28        // Configuration Register 3
#define CNF2        0x29        // Configuration Register 2
#define CNF1        0x2A        // Configuration Register 1
#define CANINTE     0x2B        // CAN Interrupt Enable Register
#define CANINTF     0x2C        // CAN Interrupt Flag Register
#define EFLG        0x2D        // Error Flag Register
#define TXB0CTRL    0x30        // Transmit Buffer 0 Control Register
#define TXB1CTRL    0x40        // Transmit Buffer 1 Control Register
#define TXB2CTRL    0x50        // Transmit Buffer 2 Control Register
#define RXB0CTRL    0x60        // Receive Buffer 0 Control Register
#define RXB1CTRL    0x70        // Reveive Buffer 1 Control Register

/* * * * * * * * * * * * * * * */
/*                             */
/*    Register Bit Positions   */
/*                             */
/* * * * * * * * * * * * * * * */

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

typedef struct {
    uint16_t sid : 11;   // Standard Identifier
    uint8_t srr : 1;    // Standard Frame Remote Transmit Request
    uint8_t ide : 1;    // Extended Identifier
    uint8_t u0  : 1;    // Unimplemented
    uint32_t eid : 18;   // Extended Identifier
    uint8_t u1  : 1;    // Unimplemented
    uint8_t rtr : 1;    // Extended Frame Remote Transmission Request
    uint8_t rb1 : 1;    // Reserved Bit 1
    uint8_t rb2 : 1;    // Reserved Bit 2
    uint8_t dlc : 4;    // Data Length Code
    uint8_t data[8];    // Data

} mcp_can_frame;

/* * * * * * * * * * * * * * * */
/*                             */
/*    Function Definitions     */
/*                             */
/* * * * * * * * * * * * * * * */

bool mcp_init (uint8_t baud_prescaler);

bool mcp_check_message ();

bool mcp_get_message (mcp_can_frame * frame);

bool mcp_send_message (uint16_t id, uint8_t length, uint8_t * data);

uint8_t mcp_read (uint8_t addr);

void mcp_rts (uint8_t txb_flags);

uint8_t mcp_read_status ();

void mcp_write (uint8_t addr, uint8_t data);

void mcp_bit_modify (uint8_t addr, uint8_t mask, uint8_t data);

#endif  // MCP2515_H