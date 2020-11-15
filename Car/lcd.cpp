#include <avr/io.h>
#include "pins.h"
#include "register.h"
#include "lcd.h"

/*
 * LCD Send Data
 * 
 * Just toggles the enable bit,
 * thus sending whatever was in
 * the Data bits
 */
void lcd_send () {
    // Turn the Enable pin on
    SET(P_EN);
    // Turn the Enable pin off
    RESET(P_EN);
}

void lcd_send_nyble (uint8_t data) {
    // Set the data pins accordingly
    // TODO: Find a cleaner way to do this
    (data & 0b1000) ? SET(P_D4) : RESET(P_D4);
    (data & 0b0100) ? SET(P_D5) : RESET(P_D5);
    (data & 0b0010) ? SET(P_D6) : RESET(P_D6);
    (data & 0b0001) ? SET(P_D7) : RESET(P_D7);
    lcd_send();
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
    RESET(P_RS);
    lcd_send_nyble(0x2);

    // Now that the screen is in 4-bit mode,
    // resend the completed Function Set
    // setting screen to "2" 5x10 rows
    lcd_send_byte(0x21);

    // Set Display control
    // Dissplay on
    // Cursore om
    // Blinking on
    lcd_send_byte(0x0F);

    // Set Entry Mode
    // Cursor increments
    // Screen scrolls left
    lcd_send_byte(0x07);

    // Ready to write data
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
    lcd_send_byte(0x1);
}

void lcd_home () {
    // Sending an instruction
    RESET(P_RS);
    // Home instruction
    lcd_send_byte(0x02);
}