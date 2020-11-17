#ifndef LCD_H
#define LCD_H

/* * * * * * * * * * * * * * * */
/*                             */
/*      DELAY DEFINITIONS      */
/*                             */
/* * * * * * * * * * * * * * * */
#define LCD_START_DELAY           15
#define LCD_4BIT_DELAY            4.1
#define LCD_4BIT_DELAY_FINAL      100
#define LCD_SEND_DELAY            40
#define LCD_CMD_DELAY             2

/* * * * * * * * * * * * * * * */
/*                             */
/*    Function Definitions     */
/*                             */
/* * * * * * * * * * * * * * * */
void lcd_init();

void lcd_clear();

void lcd_home();

#endif // LCD_H