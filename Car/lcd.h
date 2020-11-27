#ifndef LCD_H
#define LCD_H

#define LCD_ROW(x) (((x % 4) % 2) * 0x40) \
                 + (((x % 4) / 2) * 0x14)

/* * * * * * * * * * * * * * * */
/*                             */
/*      Delay Definitions      */
/*                             */
/* * * * * * * * * * * * * * * */
#define LCD_START_DELAY         15
#define LCD_4BIT_DELAY          4.1
#define LCD_4BIT_DELAY_FINAL    100
#define LCD_SEND_DELAY          40
#define LCD_CMD_DELAY           2

/* * * * * * * * * * * * * * * */
/*                             */
/*         LCD Commands        */
/*                             */
/* * * * * * * * * * * * * * * */
#define LCD_FUNCTION_SET        0x20
#define LCD_8DATA_BITS          0x10
#define LCD_DISPLAY_LINES       0x08
#define LCD_CHAR_FONT           0x04

#define LCD_DISPLAY_CONTROL     0x08
#define LCD_DISPLAY_ON          0x04
#define LCD_CURSOR_ON           0x02
#define LCD_BLINKING_ON         0x01

#define LCD_ENTRY_MODE          0x04
#define LCD_INC_CURSOR          0x02
#define LCD_SHIFT_DISPLAY       0x01

#define LCD_SHIFT               0x10
#define LCD_DISPLAY_SHIFT       0x08
#define LCD_SHIFT_RIGHT         0x04

#define LCD_SET_CGRAM           0x40

#define LCD_SET_DDRAM           0x80

#define LCD_ROW_ONE             0x00
#define LCD_ROW_TWO             0x40
#define LCD_ROW_THREE           0x14
#define LCD_ROW_FOUR            0x54

#define LCD_CLEAR               0x01
#define LCD_HOME                0x02

/* * * * * * * * * * * * * * * */
/*                             */
/*    Character Definitions    */
/*                             */
/* * * * * * * * * * * * * * * */
#define LCD_R_ARROW             0x7E    // →
#define LCD_L_ARROW             0x7F    // ←
#define LCD_BULLET              0x15

/* * * * * * * * * * * * * * * */
/*                             */
/*    Function Definitions     */
/*                             */
/* * * * * * * * * * * * * * * */
void lcd_init ();
void lcd_clear ();
void lcd_home ();
void lcd_set_cursor (uint8_t addr);
void lcd_cursor_left ();
void lcd_cursor_down ();
void lcd_putc (char c);
void lcd_print (const char * s);

#endif // LCD_H