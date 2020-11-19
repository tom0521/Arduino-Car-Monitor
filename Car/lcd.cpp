#include <avr/io.h>
#include <util/delay.h>
#include "pins.h"
#include "register.h"
#include "lcd.h"

/*
 * LCD Pulse Enable
 * 
 * Just toggles the enable bit,
 * thus sending whatever was in
 * the Data bits
 */
void lcd_pulse_enable () {
    // Make sure the enable pin starts off
    RESET(P_EN);
    // Turn the Enable pin on
    SET(P_EN);
    // Turn the Enable pin off
    RESET(P_EN);
    _delay_us(LCD_SEND_DELAY);
}

void lcd_send_nyble (uint8_t data) {
    // Set the data pins accordingly
    // TODO: Find a cleaner way to do this
    if ((data & 0x08) != 0) {
        SET(P_D7);
    } else {
        RESET(P_D7);
    }
    if ((data & 0x04) != 0) {
        SET(P_D6);
    } else {
        RESET(P_D6);
    }
    if ((data & 0x02) != 0) {
        SET(P_D5);
    } else {
        RESET(P_D5);
    }
    if ((data & 0x01) != 0) {
        SET(P_D4);
    } else {
        RESET(P_D4);
    }
    lcd_pulse_enable();
}

void lcd_send_byte (uint8_t data) {
    lcd_send_nyble(data >> 4);
    lcd_send_nyble(data);
}

/*
 * LCD Initialize
 * 
 * Sets the lcd up with the given row 
 * and column counts.
 */
void lcd_init () {
    // Set the LCD into 4-bit data length mode
    // based off of the datasheet's instructions
    RESET(P_RS);
    RESET(P_EN);
    _delay_ms(LCD_START_DELAY);
    lcd_send_nyble(0x3);
    _delay_ms(LCD_4BIT_DELAY);
    lcd_send_nyble(0x3);
    _delay_us(LCD_4BIT_DELAY);
    lcd_send_nyble(0x3);
    _delay_ms(LCD_4BIT_DELAY_FINAL);
    lcd_send_nyble(0x2);

    // Now that the screen is in 4-bit mode,
    // resend the completed Function Set
    // setting screen to "2" 5x10 rows
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
    RESET(P_RS);
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
    RESET(P_RS);
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
    RESET(P_RS);
    // Send the set DDRAM command with
    // the address to move to
    lcd_send_byte(LCD_SET_DDRAM | addr);
}

void lcd_cursor_left () {
    // Sending a command byte
    RESET(P_RS);
    // Just send command to move cursor left
    lcd_send_byte(LCD_SHIFT);
}

void lcd_cursor_down () {
    // Sending a command byte
    RESET(P_RS);
    // Move the cursor ahead 40 times
    // to move it down one row
    for (uint8_t i = 0; i < 40; ++i) {
        lcd_send_byte(LCD_SHIFT | LCD_SHIFT_RIGHT);
    }
}

/*
 * LCD Print String
 * 
 * Prints the given string to the
 * current cursor position
 */
void lcd_print (const char * s) {
    // Set the Register Select pin for
    // writing data to DDRAM
    SET(P_RS);
    // Print every character in the string
    // stopping at the null character
    for ( ; *s != '\0'; ++s) {
        lcd_send_byte(*s);
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
    SET(P_RS);
    // Print the character
    lcd_send_byte(c);
}