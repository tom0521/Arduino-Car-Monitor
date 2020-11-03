/*
  Car Arduino Project

  an attempt to not use libraries and actually read datasheets
  
*/

// TODO: remove
#include "defauts.h"
#include "register.h"
#include "mcp2515.h"
#include "spi.h"
#include <LiquidCrystal.h>

/* LCD pin definitions */
#define RS 14
#define EN 15
#define D4 16
#define D5 17
#define D6 18
#define D7 19

/* byte of flags needed for this project */
uint8_t flags = 0;  // (R_FLAG)(B_FLAG)(G_FLAG)(CAN_FLAG)
volatile int encoder = 0;
volatile uint8_t rotary_state = 0;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

/**
 * Setup
 * 
 * Initializes:
 *   1. Rotary Encoder Button
 *   2. Rotary Encoder A & B
 *   3. Rotary Encoder LEDs
 *   4. LCD Screen
 *   5. CAN Bus connection
 *   
 * Checks:
 *   1. CAN Bus Connection
 *   2. Available OBD-II PIDs
 */
void setup() {
  // Start serial connection for debugging
  Serial.begin(9600);
  
  /* * * * * * * * * * * * * * * * * * * * * */
  /*       Rotary Encoder Button setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Set the rotatry encoder button to be input no pull up resistor
  // DDRD &= ~(1 << SW);
  SET_INPUT(P_SW);
  
  // Falling edge of INT0 generates interrupt
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);
  // Enable interrupts for INT0
  EIMSK |= (1 << INT1);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder A & B setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Set the rotary encoder A pin to be input with the pull up resistor
  SET_INPUT(P_A);
  SET(P_A);

  // Set the rotary encoder B pin to be input with the pull up resistor
  SET_INPUT(P_B);
  SET(P_B);

  // Eable interrupts on the rotary encoder A pin
  PCMSK0 |= (1 << PCINT0);
  // Enable PORTB Pin change interrupts
  PCICR |= (1 << PCIE0);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder LEDs Setup        */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Set all of the LED pins as output pins
  // DDRD |= (1 << R_LED) | (1 << G_LED) | (1 << B_LED);
  SET_OUTPUT(R_LED);
  SET_OUTPUT(G_LED);
  SET_OUTPUT(B_LED);
  // Turn off all LEDs by setting them logical HIGH
  // PORTD |= (1 << R_LED) | (1 << G_LED) | (1 << B_LED);
  SET(R_LED);
  SET(G_LED);
  SET(B_LED);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*                 LCD setup               */
  /* * * * * * * * * * * * * * * * * * * * * */
  lcd.begin(20,4);
  
  /* * * * * * * * * * * * * * * * * * * * * */
  /*                 SPI setup               */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Initialize SPI
  spi_init();
  
  /* * * * * * * * * * * * * * * * * * * * * */
  /*               CAN-BUS Setup             */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Initialize CAN-BUS communication
  if (!mcp_init(0x01)) {
    Serial.println("Failed");
  } else {
    Serial.println("Success");
  }

  // PIDs 0x5E, 0x0D, 0x2F, 0xA6 needed
  
  // Turn on global interrupts
  sei();
}

/**
 * Main loop
 * 
 * Logic flow:
 *   1. 
 *   2. 
 */
void loop() {
  lcd.setCursor(0,0);
  lcd.print(encoder);
  // Serial.println(encoder);
}

/* * * * * * * * * * * * * * * * * * * * * */
/*        Interrupt Service Routines       */
/* * * * * * * * * * * * * * * * * * * * * */
/**
 * Interrupt service routine for the rotary
 * encoder button. Called on the fallin edge
 * of a button press.
 * 
 * Upon being called, this function will
 * reset the encoder variable back to zero.
 */
ISR(INT1_vect) {
  // Set the current menu to the new menu
  encoder = 0;
}

/**
 * Interrupt service routine for pin changes
 * on PORTB. This is called when there is a
 * change in position to the rotary encoder.
 * 
 * Using the current and previous encoder
 * positions, this function will increment or
 * decrement the encoder variable.
 */
ISR(PCINT0_vect) {

  rotary_state <<= 2;
  rotary_state |= (IS_SET(P_A) << 1) | (IS_SET(P_B));
  rotary_state &= 0x0F;

  if (rotary_state == 0x09) {
    ++encoder;
  }
  else if (rotary_state == 0x03) {
    --encoder;
  }
}
