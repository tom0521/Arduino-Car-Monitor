/*
  Car Arduino Project

  an attempt to not use libraries and actually read datasheets
  
*/

// TODO: remove
#include "defauts.h"
#include "register.h"
#include "mcp2515.h"
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
  // DDRD &= ~(1 << A);
  SET_INPUT(P_A);
  // PORTD |= (1 << A);
  SET(P_A);

  // Set the rotary encoder B pin to be input with the pull up resistor
  // DDRB &= ~(1 << B);
  SET_INPUT(P_B);
  // PORTB |= (1 << B);
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
  // Set all chip selects HIGH and as output
  SET(MCP_CS);
  // Set the MCP2515 Chip Select pin as output
  SET_OUTPUT(MCP_CS);

  // Set MOSI and SCK as outputs
  // PORTB |= (1 << MOSI) | (1 << SCK);
  SET_OUTPUT(P_MOSI);
  SET_OUTPUT(P_SCK);

  // Set MISO as input
  // PORTB &= ~(1 << MISO);
  SET_INPUT(P_MISO);
  // Enable SPI, Master, set clock rate fck/16
  SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (1 << SPR0);
  SPSR = 0;
  
  /* * * * * * * * * * * * * * * * * * * * * */
  /*               CAN-BUS Setup             */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Set up MCP2515 interrupt
  SET_INPUT(MCP_INT);
  SET(MCP_INT);

  // Reset MCP2515
  RESET(MCP_CS);
  spi_transmit(MCP_RESET);   // RESET instruciton
  SET(MCP_CS);

  _delay_us(10);

  // Initialize the CAN-BUS, printing a message upon failure
  RESET(MCP_CS);
  spi_transmit(MCP_WRITE);   // Transmit WRITE instruction
  spi_transmit(CNF3);   // Start on CNF3 Register
  spi_transmit(1 << PHSEG21);   // Set PS2 length = (2 + 1) * Tq
                        // Move to CNF2 Register
  spi_transmit((1 << BTLMODE) | (1 << PHSEG11));  // BTLMODE 1 (PS2 len determined by CNF3) and PS1 length = (2 + 1) *Tq
                        // Move to CNF1 Register
  spi_transmit(0x01);   // Set the baud rate prescaler to 1 (Tq = 2 * (baud_rate + 1)/Fosc )
                        // Move to CANINTE Register
  spi_transmit((1 << RX0IE) | (1 << RX1IE));   // activate interrupts
  SET(MCP_CS);

  // On successful initialization, check the available PIDs
  uint8_t data;
  RESET(MCP_CS);
  spi_transmit(MCP_READ);   // READ instruction
  spi_transmit(CNF1);   // CNF1 Register
  data = spi_transmit(0x00);
  SET(MCP_CS);

  if (data != 0x01) {
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

/**
 * SPI transmit function
 * 
 * Will send the given byte of data over
 * the Serial Parallel Interface and return
 * the byte received, if any
 * 
 */
uint8_t spi_transmit(uint8_t data) {
  // Start transmitting data to slave
  SPDR = data;

  // Wait for transmission to complete
  while (!(SPSR & (1 << SPIF)))
    ;

  return SPDR;
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
  static unsigned char rotary_state = 0;

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
