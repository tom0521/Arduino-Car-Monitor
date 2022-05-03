#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

#include "encoder.h"
#include "lcd.h"
#include "list.h"
#include "mcp2515.h"
#include "obd2.h"
#include "pins.h"
#include "register.h"
#include "spi.h"

#define SPI_ERR       0x01
#define SUPPORT_ERR   0x20

/* Print error code to screen */
void print_error (uint8_t error) {
  lcd_set_cursor(LCD_POS(3,14));
  lcd_sprintf("%#x",error);
}

void main() {
  uint32_t support;
  float speed;
  float fuel_consumption;
  float mpg;

  /* SETUP */
  lcd_init();
  encoder_init();
  lcd_set_cursor(LCD_POS(1,2));
  lcd_sprintf("Starting Up...");
  _delay_ms(1000);
  lcd_clear();

  if (!obd2_init()) {
    print_error(SPI_ERR);
    return;
  }

  support = obd2_read_pid(OBD2_PID_SUPPORT_1).bits;
  if (!(support & ((uint32_t)1 << (32 - OBD2_PID_SUPPORT_2)))) {
    print_error(SUPPORT_ERR & 0x1);
    return;
  }
  if (!(support & ((uint32_t)1 << (32 - OBD2_VEHICLE_SPEED)))) {
    print_error(SUPPORT_ERR & 0x2);
    return;
  }
  support = obd2_read_pid(OBD2_PID_SUPPORT_2).bits;
  if (!(support & ((uint32_t)1 << (32 - (OBD2_PID_SUPPORT_3 - OBD2_PID_SUPPORT_2))))) {
    print_error(SUPPORT_ERR & 0x3);
    return;
  }
  support = obd2_read_pid(OBD2_PID_SUPPORT_3).bits;
  if (!(support & ((uint32_t)1 << (32 - (OBD2_ENGINE_FUEL_RATE - OBD2_PID_SUPPORT_3))))) {
    print_error(SUPPORT_ERR & 0x4);
    return;
  }

  /* Global interrupts on */
  sei();

  for ( ; ; ) {
    speed = obd2_read_pid(OBD2_VEHICLE_SPEED).val * 0.6213712f;
    fuel_consumption = obd2_read_pid(OBD2_ENGINE_FUEL_RATE).val * 0.2641729f;
    mpg = fuel_consumption ? speed / fuel_consumption : 0.0f; 
    lcd_set_cursor(LCD_POS(1,6));
    lcd_sprintf("%3u mpg", (int) mpg);
    _delay_ms(100);
  }
}
