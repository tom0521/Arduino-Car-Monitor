#ifndef ROTENC_H
#define ROTENC_H

#define LCD_ROW(x) (((x % 4) % 2) * 0x40) \
                 + (((x % 4) / 2) * 0x14)

/* * * * * * * * * * * * * * * */
/*                             */
/*      Delay Definitions      */
/*                             */
/* * * * * * * * * * * * * * * */
#define LCD_START_DELAY         15
#define LCD_MIDDLE_DELAY        4.1
#define LCD_FINAL_DELAY         100
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
void rotenc_init ();

#endif // ROTENC_H
