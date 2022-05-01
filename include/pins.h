#ifndef PINS_H
#define PINS_H

/* Rotary Encoder Pins */
#define ENC_SW      D,3     // Pin 3
#define ENC_LED_R   D,4     // Pin 4
#define ENC_LED_G   D,5     // Pin 5
#define ENC_LED_B   D,6     // Pin 6
#define ENC_A       D,7     // Pin 7
#define ENC_B       B,0     // Pin 8

/* LCD Configuration */
#define _LCD_4BIT           // Only use 4 data bits

/* LCD Pins */
#define LCD_RS      C,0     // Pin 14
#define LCD_EN      C,1     // Pin 15
#define LCD_D4      C,2     // Pin 16
#define LCD_D5      C,3     // Pin 17
#define LCD_D6      C,4     // Pin 18
#define LCD_D7      C,5     // Pin 19

/* MCP2515 configuration */
#define _MCP_SPI            // Initialize SPI

/* MCP2515 interrupt */
#define MCP_CS      B,2     // Pin 10
#define MCP_INT     D,2     // Pin 2

/* SPI Pins */
#define SPI_MOSI    B,3     // Pin 11
#define SPI_MISO    B,4     // Pin 12
#define SPI_SCK     B,5     // Pin 13

#endif // PINS_H
