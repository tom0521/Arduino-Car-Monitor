#ifndef PINS_H
#define PINS_H

                        // Digital Pins:
/* Rotary Encoder Pins */
#define P_SW    D,3     // Pin 3
#define R_LED   D,4     // Pin 4
#define G_LED   D,5     // Pin 5
#define B_LED   D,6     // Pin 6
#define P_A     D,7     // Pin 7
#define P_B     B,0     // Pin 8

/* LCD Pins */
#define P_D7    C,0     // Pin 14
#define P_D6    C,1     // Pin 15
#define P_D5    C,2     // Pin 16
#define P_D4    C,3     // Pin 17
#define P_RS    C,4     // Pin 18
#define P_EN    C,5     // Pin 19

/* SPI Pins */
#define MCP_CS  B,2     // Pin 10
#define P_MOSI  B,3     // Pin 11
#define P_MISO  B,4     // Pin 12
#define P_SCK   B,5     // Pin 13

/* MCP2515 interrupt */
#define MCP_INT D,2     // Pin 2

#endif // PINS_H
