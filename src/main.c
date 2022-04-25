#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "lcd.h"
#include "list.h"
#include "mcp2515.h"
#include "obd2.h"
#include "pins.h"
#include "register.h"
#include "spi.h"
#include "process.h"

#define CURSOR LCD_R_ARROW

/* Current position of the encoder */
volatile uint8_t encoder = 0;

/*
 * Process Queue
 * 
 * Contains jobs waiting to be processed.
 * No more than 10 jobs should be created which
 * is 90 bytes (3 x 3 byte pointers per process)
 * created using malloc. This means that the
 * dynamic space available is pleanty
 */
struct list queue;

/**
 * Setup
 * 
 * Initializes:
 *   1. Rotary Encoder Button
 *   2. Rotary Encoder A & B
 *   3. Rotary Encoder LEDs
 *   4. LCD Screen
 *   5. CAN Bus connection
 *   6. Send catalyst
 */
void setup() {
  /* * * * * * * * * * * * * * * * * * * * * */
  /*                 I/O setup               */
  /* * * * * * * * * * * * * * * * * * * * * */
  /* Rotary Encoder */
  // Set the rotatry encoder button to be input no pull up resistor
  SET_INPUT(P_SW);
  // Set the rotary encoder A pin to be input with the pull up resistor
  SET_INPUT(P_A);
  SET(P_A);
  // Set the rotary encoder B pin to be input with the pull up resistor
  SET_INPUT(P_B);
  SET(P_B);
  // Set all of the LED pins as output pins
  SET_OUTPUT(R_LED);
  SET_OUTPUT(G_LED);
  SET_OUTPUT(B_LED);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*       Rotary Encoder Button setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Falling edge of INT1 generates interrupt
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);
  // Enable interrupts for INT1
  EIMSK |= (1 << INT1);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder A & B setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Enable interrupts on the rotary encoder A pin
  PCMSK0 |= (1 << PCINT0);
  // Enable PORTB Pin change interrupts
  PCICR |= (1 << PCIE0);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder LEDs Setup        */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Turn off all LEDs by setting them logical HIGH
  SET(R_LED);
  SET(G_LED);
  SET(B_LED);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*                 LCD setup               */
  /* * * * * * * * * * * * * * * * * * * * * */
  lcd_init();

  /* * * * * * * * * * * * * * * * * * * * * */
  /*                 SPI setup               */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Initialize SPI
  spi_init();
  
  /* * * * * * * * * * * * * * * * * * * * * */
  /*               CAN-BUS Setup             */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Falling edge of INT0 generates interrupt
  // EICRA |= (1 << ISC01);
  // EICRA &= ~(1 << ISC00);
  // Enable interrupts for INT0
  // EIMSK |= (1 << INT0);

  // Initialize CAN-BUS communication
  if (!mcp_init(0x01)) {
    // Do something because it failed
  }
  /*
  if (!SD.begin(9)) {
    // Do something becuase ot failed
    Serial.println("SD Failed");
  } else {
    Serial.println("SD SUCC");
  }*/

  // Turn on global interrupts
  sei();

  lcd_set_cursor(LCD_ROW(0));
  lcd_print("Range: ----");
}

uint64_t N = 0;

double init_miles = 0;

double sum_x = 0,
      sum_y = 0,
      sum_x2 = 0,
      sum_xy = 0;

/**
 * Main loop  
 */
void loop() {
  // // Get a base relative time
  // unsigned long d_time = millis();
  // // Read the speed of the vehicle
  // double speed = obd_read_pid(OBD_VEHICLE_SPEED);
  // // Get a change in relative time
  // unsigned long d_time2 = millis();
  // if (d_time > d_time2) {
  //   d_time = (((unsigned long) -1) - d_time) + d_time2;
  // } else {
  //   d_time = d_time2 - d_time;
  // }
  // // Get the miles traveled (roughly)
  // miles += (speed * (d_time / 3.6e6));
  double miles = obd2_read_pid(OBD2_DIST_CODE_CLR);
  if (init_miles == 0) {
    init_miles = miles;
  }
  miles -= init_miles;
  // Read the Fuel %
  double fuel = obd2_read_pid(OBD2_FUEL_TANK_LEVEL);


  ++N;
  sum_x += fuel;
  sum_y += miles;
  sum_x2 += (fuel * fuel);
  sum_xy += (fuel * miles);
  
  double denom = ((N * sum_x2) - (sum_x * sum_x));
  // double m = ((N * sum_xy) - (sum_x * sum_y)) / denom;
  double b = ((sum_y * sum_x2) - (sum_x * sum_xy)) / denom;

  double range = b - miles;
  if (range > 0 && range < 500) {
    lcd_set_cursor(LCD_ROW(0)+7);
    lcd_printf(range);
  }
  // lcd_set_cursor(LCD_ROW(1));
  // lcd_print(speed); lcd_print(" mph");
  lcd_set_cursor(LCD_ROW(2));
  lcd_printf(fuel); lcd_print("%");
  lcd_set_cursor(LCD_ROW(3));
  lcd_printf(miles); lcd_print(" miles");

  /* File log = SD.open("fuel.csv", FILE_WRITE);
  log.print(fuel); log.print(','); log.println(miles);
  log.close(); */
}

/* * * * * * * * * * * * * * * * * * * * * */
/*        Interrupt Service Routines       */
/* * * * * * * * * * * * * * * * * * * * * */

/**
 * Interrup service routine for MCP2515.
 * This is called whenever the pin is driven
 * high.
 * 
 * Once called, a process will be added to the
 * queue to process received messages
 */
//ISR(INT0_vect) {
  // Add a get message process to the job queue
  /** TODO: Need semaphore or some kind of synchronization */
  // struct process * p = (struct process *) malloc(sizeof(*p));
  // p->func = obd_response;
  // list_enqueue(&queue, &p->elem);
//}

/*
 * Interrupt service routine for the rotary
 * encoder button. Called on the fallin edge
 * of a button press.
 * 
 * Upon being called, this function will
 * reset the encoder variable back to zero.
 */
/*ISR(INT1_vect) {
  // Add a select handler to the queue
  encoder = 0;
}*/

/*
 * Interrupt service routine for pin changes
 * on PORTB. This is called when there is a
 * change in position to the rotary encoder.
 * 
 * Using the current and previous encoder
 * positions, this function will increment or
 * decrement the encoder variable.
 */
/*ISR(PCINT0_vect) {
  static uint8_t rotary_state = 0;
  rotary_state <<= 2;
  rotary_state |= ((PIND >> 6) & 0x2) | ((PINB >> 0) & 0x1);
  rotary_state &= 0x0F;

  if (rotary_state == 0x9) {
    ++encoder;
  }
  else if (rotary_state == 0x3) {
    --encoder;
  }
}*/

void main() {
    /* Initialize I/O */
    /* LCD */
    /* Rotary Encoder */
    /* MCP2515 */
    setup();
    for ( ; ; )
        loop();
}
