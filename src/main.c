#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "encoder.h"
#include "lcd.h"
#include "list.h"
#include "mcp2515.h"
#include "obd2.h"
#include "pins.h"
#include "register.h"
#include "spi.h"

#define SPI_ERR   0x01
#define SPEED_ERR 0x02
#define FUEL_ERR  0x03

/* Print error code to screen */
void print_error (uint8_t error) {
  lcd_set_cursor(LCD_POS(3,16));
  lcd_sprintf("0x%x",error);
}

void main() {
  float speed;
  float fuel_consumption;
  float km_per_liter;

  /* SETUP */
  lcd_init();
  encoder_init();
  if (!obd2_init())
    print_error(SPI_ERR);
  
  /* Global interrupts on */
  sei();
  
  for ( ; ; ) {
    if ((speed = obd2_read_pid(OBD2_VEHICLE_SPEED)) == -1) {
      print_error(SPEED_ERR);
      break;
    }
    if ((fuel_consumption = obd2_read_pid(OBD2_ENGINE_FUEL_RATE)) == -1) {
      print_error(FUEL_ERR);
      break;
    }
    km_per_liter = fuel_consumption ? speed / fuel_consumption : 0.0; 
    lcd_set_cursor(LCD_POS(0,0));
    lcd_sprintf("%3u km/L", (int) km_per_liter);
  }
}
