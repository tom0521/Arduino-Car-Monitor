#include "pins.h"
#include "register.h"
#include "rotenc.h"

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
#ifndef LCD_4BIT
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
#ifdef LCD_4BIT
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
