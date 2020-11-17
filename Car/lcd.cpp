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
    lcd_send_byte(0x28);

    // Set Display control
    // Dissplay on
    // Cursore om
    // Blinking on
    lcd_send_byte(0x0F);

    // Set Entry Mode
    // Cursor increments
    // Screen scrolls left
    lcd_send_byte(0x06);

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
    lcd_send_byte(0x01);
    _delay_ms(LCD_CMD_DELAY);
}

void lcd_home () {
    // Sending an instruction
    RESET(P_RS);
    // Home instruction
    lcd_send_byte(0x02);
    _delay_ms(LCD_CMD_DELAY);
}