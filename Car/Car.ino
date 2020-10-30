/*
  Car Arduino Project
*/

#include <LiquidCrystal.h>

/* Rotary Encoder pin definitions */
#define SW PD3     // Pin 3
#define R_LED PD4  // Pin 4
#define G_LED PD5  // Pin 5
#define B_LED PD6  // Pin 6
#define A PD7      // Pin 7
#define B PB0      // Pin 8

/* LCD pin definitions definitions */
#define RS 14
#define EN 15
#define D4 16
#define D5 17
#define D6 18
#define D7 19

volatile int encoder = 0;

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
  // Serial.begin(9600);
  
  /* * * * * * * * * * * * * * * * * * * * * */
  /*       Rotary Encoder Button setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Set the rotatry encoder button to be input no pull up resistor
  DDRD &= ~(1 << SW);
  PORTD &= ~(1 << SW);
  
  // Falling edge of INT0 generates interrupt
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);

  // Enable interrupts for INT0
  EIMSK |= (1 << INT1);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder A & B setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Set the rotary encoder A pin to be input with the pull up resistor
  DDRD &= ~(1 << A);
  PORTD |= (1 << A);

  // Set the rotary encoder B pin to be input with the pull up resistor
  DDRB &= ~(1 << B);
  PORTB |= (1 << B);

  // Eable interrupts on the rotary encoder A pin
  PCMSK0 |= (1 << PCINT0);

  // Enable PORTB Pin change interrupts
  PCICR |= (1 << PCIE0);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder LEDs Setup        */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Set all of the LED pins as output pins
  DDRD |= (1 << R_LED) | (1 << G_LED) | (1 << B_LED);
  // Turn off all LEDs by setting them logical HIGH
  PORTD |= (1 << R_LED) | (1 << G_LED) | (1 << B_LED);
  
  // Turn on global interrupts
  sei();

  lcd.begin(20,4);
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
  static unsigned char rotary_state = 0;

  rotary_state <<= 2;
  rotary_state |= (((PINB >> PINB0) & 1) | (((PIND >> PIND7) & 1) << 1));
  rotary_state &= 0x0F;

  if (rotary_state == 0x09) {
    ++encoder;
  }
  else if (rotary_state == 0x03) {
    --encoder;
  }
}
