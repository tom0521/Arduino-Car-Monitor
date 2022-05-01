#include <avr/io.h>
#include <util/delay.h>
#include <stdarg.h>
#include <stdlib.h>
#include "pins.h"
#include "register.h"
#include "util.h"
#include "lcd.h"

/* * * * * * * * * * * * * * * */
/*      LCD Printf flags       */
/* * * * * * * * * * * * * * * */
#define LCD_LEFT_JUSTIFY    0x80
#define LCD_SIGN            0x40
#define LCD_BLANK           0x20
#define LCD_POUND           0x10
#define LCD_ZERO_PADDING    0x08
#define LCD_UPPERCASE       0x04

#define LCD_SPECIFIER       0x01

const uint8_t int_nybles = sizeof(int) * 2;
/*
 * LCD Pulse Enable
 * 
 * Just toggles the enable bit,
 * thus sending whatever was in
 * the Data bits
 */
void lcd_pulse_enable () {
    // Make sure the enable pin starts off
    RESET(LCD_EN);
    // Turn the Enable pin on
    SET(LCD_EN);
    // Turn the Enable pin off
    RESET(LCD_EN);
    _delay_us(LCD_SEND_DELAY);
}

/*
 * LCD Send Nyble
 * 
 * Sends a nyble of data to the
 * LCD by setting the correct bits
 * and then pulsing the enable pin
 */
void lcd_send_nyble (uint8_t data) {
    // Set all of the data bits accordingly
    (data & 0b0001) ? SET(LCD_D4) : RESET(LCD_D4);
    (data & 0b0010) ? SET(LCD_D5) : RESET(LCD_D5);
    (data & 0b0100) ? SET(LCD_D6) : RESET(LCD_D6);
    (data & 0b1000) ? SET(LCD_D7) : RESET(LCD_D7);
    
    // Send the data
    lcd_pulse_enable();
}

/*
 * LCD Send Byte
 * 
 * Sends one byte of data to the LCD
 */
void lcd_send_byte (uint8_t data) {
#ifdef _LCD_4BIT
    lcd_send_nyble(data >> 4);
    lcd_send_nyble(data);
#else
    (data & 0b00000001) ? SET(LCD_D0) : RESET(LCD_D0);
    (data & 0b00000010) ? SET(LCD_D1) : RESET(LCD_D1);
    (data & 0b00000100) ? SET(LCD_D2) : RESET(LCD_D2);
    (data & 0b00001000) ? SET(LCD_D3) : RESET(LCD_D3);
    (data & 0b00010000) ? SET(LCD_D4) : RESET(LCD_D4);
    (data & 0b00100000) ? SET(LCD_D5) : RESET(LCD_D5);
    (data & 0b01000000) ? SET(LCD_D6) : RESET(LCD_D6);
    (data & 0b10000000) ? SET(LCD_D7) : RESET(LCD_D7);
    
    // Send the data
    lcd_pulse_enable();
#endif
    
}

/*
 * LCD Initialize
 * 
 * Sets the lcd up with the given row 
 * and column counts.
 */
void lcd_init () {
    // Set Register Select pin as output
    SET_OUTPUT(LCD_RS);
    // Set enable pin as output
    SET_OUTPUT(LCD_EN);
    // Set all the data pins as output
#ifndef _LCD_4BIT
    SET_OUTPUT(LCD_D0);
    SET_OUTPUT(LCD_D1);
    SET_OUTPUT(LCD_D2);
    SET_OUTPUT(LCD_D3);
#endif
    SET_OUTPUT(LCD_D4);
    SET_OUTPUT(LCD_D5);
    SET_OUTPUT(LCD_D6);
    SET_OUTPUT(LCD_D7);
    // Set the LCD into desired data length mode
    // based off of the datasheet's instructions
    RESET(LCD_RS);
    RESET(LCD_EN);
    _delay_ms(LCD_START_DELAY);
    lcd_send_nyble(0x3);
    _delay_ms(LCD_MIDDLE_DELAY);
    lcd_send_nyble(0x3);
    _delay_us(LCD_FINAL_DELAY);
#ifdef _LCD_4BIT
    lcd_send_nyble(0x2);
#else
    lcd_send_nyble(0x3);
#endif

    // Now that the screen is in the desired mode,
    // resend the completed Function Set to setup
    // the number of display lines
    lcd_send_byte(LCD_FUNCTION_SET | LCD_DISPLAY_LINES);

    // Set Display control
    // Dissplay on
    // Cursore om
    // Blinking on
    lcd_send_byte(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON);

    // Set Entry Mode
    // Cursor increments
    // Screen scrolls left
    lcd_send_byte(LCD_ENTRY_MODE | LCD_INC_CURSOR);

    // Start fresh
    lcd_clear();
    lcd_home();
}

/*
 * LCD Clear Screen
 * 
 * Send the clear screen
 * instruction to the LCD
 */
void lcd_clear () {
    // Sending an instruction
    RESET(LCD_RS);
    // Clear instruction
    lcd_send_byte(LCD_CLEAR);
    _delay_ms(LCD_CMD_DELAY);
}

/*
 * LCD Home
 * 
 * Sends the command to go
 * return the cursor home
 */
void lcd_home () {
    // Sending an instruction
    RESET(LCD_RS);
    // Home instruction
    lcd_send_byte(LCD_HOME);
    _delay_ms(LCD_CMD_DELAY);
}

/*
 * LCD Set Cursor
 * 
 * Sets the position of the
 * cursor on the screen
 */
void lcd_set_cursor (uint8_t addr) {
    // Sending a command byte
    RESET(LCD_RS);
    // Send the set DDRAM command with
    // the address to move to
    lcd_send_byte(LCD_SET_DDRAM | addr);
}

void lcd_cursor_left () {
    // Sending a command byte
    RESET(LCD_RS);
    // Just send command to move cursor left
    lcd_send_byte(LCD_SHIFT);
}

void lcd_cursor_down () {
    // Sending a command byte
    RESET(LCD_RS);
    // Move the cursor ahead 40 times
    // to move it down one row
    for (uint8_t i = 0; i < 40; ++i) {
        lcd_send_byte(LCD_SHIFT | LCD_SHIFT_RIGHT);
    }
}

/*
 * LCD Put Character
 * 
 * Prints the character to the
 * current cursor position
 */
void lcd_putc (char c) {
    // Set the Register Select pin for
    // writing data to DDRAM
    SET(LCD_RS);
    // Print the character
    lcd_send_byte(c);
}

/*
 * LCD Print String
 * 
 * Prints the given string to the
 * current cursor position
 */
void lcd_print (const char *s) {
    // Set the Register Select pin for
    // writing data to DDRAM
    SET(LCD_RS);
    // Print every character in the string
    // stopping at the null character
    for ( ; *s != '\0'; ++s) {
        lcd_send_byte(*s);
    }
}

/*
 * LCD Print Unsigned Integer
 *
 * Prints given unsigned integer
 * with the given width
 */
void lcd_printu (unsigned int d, uint8_t width) {
  // Think positively for now
  char buff[5];
  uint8_t i;
  uint8_t length;

  for (i = 5; i > 0 && (d | i == 5); --i) {
    buff[i-1] = 0x30 + (d % 10);
    d /= 10;
  }

  // Set the Register Select pin for
  // writing data to DDRAM
  SET(LCD_RS);
  for (length = 0; length + 5 - i < width; ++length)
    lcd_send_byte(' ');
  // Print the buffer
  for ( ; i < 5; ++i)
    lcd_send_byte(buff[i]);
}

/*
 * LCD Print Hex
 *
 * Prints given integer in
 * hexadecimal
 */
void lcd_printx (unsigned int x, uint8_t width, uint8_t flags) {
  uint8_t  *ptr;
  uint16_t hex;

  // Set the Register Select pin for
  // writing data to DDRAM
  SET(LCD_RS);
  
  if (flags & LCD_POUND) {
    lcd_send_byte('0');
    lcd_send_byte((flags & LCD_UPPERCASE) ? 'X' : 'x');
  }
  // TODO: padding

  // Loop through each byte of the integer (Little Endian)
  for (ptr = (uint8_t *)&x + sizeof(int) - 1; ptr >= (uint8_t *)&x; --ptr) {
    hex = hex2ascii(*ptr, flags & LCD_UPPERCASE);
    lcd_send_byte((uint8_t)(hex>>8));
    lcd_send_byte((uint8_t)(hex&0xFF));
  }
}

/*
 * LCD Print formatted string
 * 
 * Prints variables in formatted
 * string to the screen
 */
void lcd_sprintf (const char *format, ...) {
  va_list ap;
  uint8_t width;
  uint8_t flags;
  
  va_start(ap, format);
  for (char *ptr = (char *)format; *ptr != '\0'; ++ptr) {
    if (*ptr == '%') {
      flags = 0;
      width = 0;
      while (!(flags & LCD_SPECIFIER)) {
        switch (*(++ptr)) {
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9': width = (width * 10) + (*ptr) - 0x30;
                    break;
          case 'd': 
          case 'i': lcd_printx(va_arg(ap, int), width, flags);
                    break;
          case 'u': lcd_printu(va_arg(ap, unsigned int), width);
                    flags |= LCD_SPECIFIER;
                    break;
          case 'X': flags |= LCD_UPPERCASE;
          case 'x': lcd_printx(va_arg(ap, unsigned int), width, flags);
                    flags |= LCD_SPECIFIER;
                    break;
          default:  lcd_putc(*ptr);
                    flags |= LCD_SPECIFIER;
                    break;
        }
      }
    } else
      lcd_putc(*ptr);
  }
  va_end(ap);
}
