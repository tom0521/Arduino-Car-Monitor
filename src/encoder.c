#include "pins.h"
#include "register.h"
#include "encoder.h"

/*
 * Rotary Encoder Initialize
 * 
 * Sets up the input and output pins
 * for the rotary encoder
 */
void encoder_init () {
  // Set the rotatry encoder button to be input no pull up resistor
  SET_INPUT(ENC_SW);
  // Set the rotary encoder A pin to be input with the pull up resistor
  SET_INPUT(ENC_A);
  SET(ENC_A);
  // Set the rotary encoder B pin to be input with the pull up resistor
  SET_INPUT(ENC_B);
  SET(ENC_B);
  // Set all of the LED pins as output pins
  SET_OUTPUT(ENC_LED_R);
  SET_OUTPUT(ENC_LED_G);
  SET_OUTPUT(ENC_LED_B);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*       Rotary Encoder Button setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Falling edge of INT1 generates interrupt
  // EICRA |= (1 << ISC11);
  // EICRA &= ~(1 << ISC10);
  // Enable interrupts for INT1
  // EIMSK |= (1 << INT1);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder A & B setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Enable interrupts on the rotary encoder A pin
  // PCMSK0 |= (1 << PCINT0);
  // Enable PORTB Pin change interrupts
  // PCICR |= (1 << PCIE0);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder LEDs Setup        */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Turn off all LEDs by setting them logical HIGH
  SET(ENC_LED_R);
  SET(ENC_LED_G);
  SET(ENC_LED_B);
}
