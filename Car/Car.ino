/*
  Car Arduino Project

  an attempt to not use libraries and actually read datasheets
  
*/

#include "pins.h"
#include "register.h"
#include "mcp2515.h"
#include "obd2.h"
#include "spi.h"
#include "lcd.h"

const static char * ERROR_MSG = ":(";
const static char * SUCC_MSG = ":)";

/* flags needed for this project */
volatile struct flags
{
  uint8_t rgb : 3;        // RGB LED color
  uint8_t can_flag : 1;   // Was CAN init successful?
  uint8_t int_flag : 1;   // Has there been user input?
} flags;

volatile uint8_t encoder = 0;

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
  /*                 I/O setup               */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Serial.println("Setting up I/O pins...");
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

  // /* LCD */
  // Set Register Select pin as output
  SET_OUTPUT(P_RS);
  // Set enable pin as output
  SET_OUTPUT(P_EN);
  // Set all the data pins as output
  // Using 4-bit mode so only using D4-D7
  SET_OUTPUT(P_D4);
  SET_OUTPUT(P_D5);
  SET_OUTPUT(P_D6);
  SET_OUTPUT(P_D7);

  /* MCP2515 */
  // Set the Chip select HIGH to not send
  // messages when spi is set up
  SET(MCP_CS);
  // Set the MCP2515 Chip Select pin as output
  SET_OUTPUT(MCP_CS);
  // Set up MCP2515 interrupt pin as input pin
  SET_INPUT(MCP_INT);
  // Enable the pull up resistor
  SET(MCP_INT);
  

  /* * * * * * * * * * * * * * * * * * * * * */
  /*       Rotary Encoder Button setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Serial.println("Setting up button interrupt...");
  // Falling edge of INT0 generates interrupt
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);
  // Enable interrupts for INT1
  EIMSK |= (1 << INT1);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder A & B setup       */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Serial.println("Setting up Encoder intrrupt...");
  // Enable interrupts on the rotary encoder A pin
  PCMSK0 |= (1 << PCINT0);
  // Enable PORTB Pin change interrupts
  PCICR |= (1 << PCIE0);


  // Serial.println("Enabling global interrupts...");
  // Turn on global interrupts
  sei();

  /* * * * * * * * * * * * * * * * * * * * * */
  /*        Rotary Encoder LEDs Setup        */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Serial.println("Turning off the LEDs...");
  // Turn off all LEDs by setting them logical HIGH
  SET(R_LED);
  SET(G_LED);
  SET(B_LED);

  /* * * * * * * * * * * * * * * * * * * * * */
  /*                 LCD setup               */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Serial.println("Setting up LCD...");
  lcd_init();
  // Initialized my Cursor
  lcd_putc(LCD_R_ARROW);
  lcd_cursor_left();

  /* * * * * * * * * * * * * * * * * * * * * */
  /*                 SPI setup               */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Serial.println("Setting up SPI interface...");
  // Initialize SPI
  spi_init();
  
  /* * * * * * * * * * * * * * * * * * * * * */
  /*               CAN-BUS Setup             */
  /* * * * * * * * * * * * * * * * * * * * * */
  // Serial.println("Setting up CAN interface...");
  // Initialize CAN-BUS communication
  if (!mcp_init(0x01)) {
    // Serial.println("CAN setup failed");
  } else { // Check for supported PIDs
    mcp_can_frame frame;
    frame.sid = CAN_ECU_REQ;
    frame.srr = 0;
    frame.ide = 0;
    frame.eid = 0;
    frame.rtr = 0;
    frame.dlc = 8;
    frame.data[OBD_LENGTH] = 0x02;
    frame.data[OBD_MODE] = OBD_CUR_DATA;
    frame.data[OBD_PID] = OBD_PID_SUPPORT_1;
    lcd_set_cursor(LCD_ROW_ONE + 2);
    if (mcp_tx_message(&frame)) {
      lcd_print("0x0D - ");
      while (!mcp_check_message())
        ;
      mcp_rx_message(&frame);
      Serial.println(frame.data[OBD_B], HEX);
      if (frame.data[OBD_B] & 0x08 != 0) {
        lcd_print(SUCC_MSG);
      }
    } else {
      lcd_print("CAN Failed");
    }

    frame.sid = CAN_ECU_REQ;
    frame.srr = 0;
    frame.ide = 0;
    frame.eid = 0;
    frame.rtr = 0;
    frame.dlc = 8;
    frame.data[OBD_LENGTH] = 0x02;
    frame.data[OBD_MODE] = OBD_CUR_DATA;
    frame.data[OBD_PID] = OBD_PID_SUPPORT_2;
    lcd_set_cursor(LCD_ROW_TWO + 2);
    if (mcp_tx_message(&frame)) {
      lcd_print("0x2F - ");
      while (!mcp_check_message())
        ;
      mcp_rx_message(&frame);
      Serial.println(frame.data[OBD_B], HEX);
      if (frame.data[OBD_B] & 0x02 != 0) {
        lcd_print(SUCC_MSG);
      }
    } else {
      lcd_print("CAN Failed");
    }

    frame.sid = CAN_ECU_REQ;
    frame.srr = 0;
    frame.ide = 0;
    frame.eid = 0;
    frame.rtr = 0;
    frame.dlc = 8;
    frame.data[OBD_LENGTH] = 0x02;
    frame.data[OBD_MODE] = OBD_CUR_DATA;
    frame.data[OBD_PID] = OBD_PID_SUPPORT_3;
    lcd_set_cursor(LCD_ROW_THREE + 2);
    if (mcp_tx_message(&frame)) {
      lcd_print("0x5E - ");
      while (!mcp_check_message())
        ;
      mcp_rx_message(&frame);
      Serial.println(frame.data[OBD_D], HEX);
      if (frame.data[OBD_D] & 0x04 != 0) {
        lcd_print(SUCC_MSG);
      }
    } else {
      lcd_print("CAN Failed");
    }
  }

  // PIDs 0x5E, 0x0D, 0x2F, 0xA6 needed
  
}

/**
 * Main loop
 * 
 * Logic flow:
 *   1. 
 *   2. 
 */
void loop() {
  mcp_can_frame frame;
  frame.sid = CAN_ECU_REQ;
  frame.dlc = 8;
  frame.data[0] = 0x02;
  frame.data[1] = OBD_CUR_DATA;
  frame.data[2] = 0x0C;

  if (flags.int_flag) {
    lcd_putc(' ');
    switch (encoder % 4)
    {
    case 0:
      lcd_set_cursor(LCD_ROW_ONE);
      break;
    case 1:
      lcd_set_cursor(LCD_ROW_TWO);
      break;
    case 2:
      lcd_set_cursor(LCD_ROW_THREE);
      break;
    case 3:
      lcd_set_cursor(LCD_ROW_FOUR);
    }
    lcd_putc(LCD_R_ARROW);
    lcd_cursor_left();
    flags.int_flag = 0;
  }
  
  lcd_set_cursor(LCD_ROW_FOUR + 2);
  // If successfully sent the message
  if (mcp_tx_message(&frame)) {
    // Serial.println("Message Sent");
    // Then wait for the reply
    while (!mcp_check_message())
      ;

    // ... and print the reply
    if (mcp_rx_message(&frame)){
      // 256A + B
      // -------- = RPM
      //    4
      Serial.print(((frame.data[OBD_A] << 8)+frame.data[OBD_B])/4.0); Serial.println(" RPM");
    }
  } else {
    // Serial.println("Message failed to send");
  }
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
  flags.int_flag = 1;
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
  static uint8_t rotary_state = 0;
  rotary_state <<= 2;
  rotary_state |= ((PIND >> 6) & 0x2) | ((PINB >> 0) & 0x1);
  rotary_state &= 0x0F;

  if (rotary_state == 0x09) {
    ++encoder;
  }
  else if (rotary_state == 0x03) {
    --encoder;
  }
  flags.int_flag = 1;
}
