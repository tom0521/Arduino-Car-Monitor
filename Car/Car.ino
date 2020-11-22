/*
  Car Arduino Project

  an attempt to not use libraries and actually read datasheets
  
*/

#include "lcd.h"
#include "list.h"
#include "mcp2515.h"
#include "obd2.h"
#include "pins.h"
#include "register.h"
#include "spi.h"
#include "process.h"

/* flags needed for this project */
volatile struct flags
{
  uint8_t rgb : 3;        // RGB LED color
  uint8_t can_flag : 1;   // Was CAN init successful?
  uint8_t int_flag : 1;   // Has there been user input?
} flags;

volatile uint8_t encoder = 0;

/*
 * Process Queue
 * 
 * Contains jobs waiting to be processed.
 * No more than 10 jobs should be created which
 * is 60 bytes (3 x 2 byte pointers per process)
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
 *   
 * Checks:
 *   1. CAN Bus Connection
 *   2. Available OBD-II PIDs
 */
void setup() {
  // Start serial connection for debugging
  // Serial.begin(9600);

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
    frame.data[OBD_FRAME_LENGTH] = 0x02;
    frame.data[OBD_FRAME_MODE] = OBD_CUR_DATA;

    // Read the first set of supported PIDs
    frame.data[OBD_FRAME_PID] = OBD_PID_SUPPORT_1;
    mcp_tx_message(&frame);

    frame.data[OBD_FRAME_PID] = OBD_PID_SUPPORT_2;
    mcp_tx_message(&frame);

    frame.data[OBD_FRAME_PID] = OBD_PID_SUPPORT_2;
    mcp_tx_message(&frame);
  }

  // PIDs 0x5E, 0x0D, 0x2F, 0xA6 needed
}

/**
 * Main loop
 * 
 * Checks if the process queue is empty
 * If there is a job to process, then remove it
 * from the queue, process it, and free it
 */
void loop() {
  // Check to see if there are any tasks to complete
  if (!list_is_empty(&queue)) {
    // Remove the process from the queue
    struct process * p = list_entry(list_dequeue(&queue), struct process, elem);
    // Execute the function
    p->func();
    // Free up the space
    free(p);
  } else {
    // Idle state
  }
}

/* * * * * * * * * * * * * * * * * * * * * */
/*        Interrupt Service Routines       */
/* * * * * * * * * * * * * * * * * * * * * */
/**
 * Interrup service routine for MCP2515.
 * This is called whenever the pin is driven
 * TODO: ??
 * 
 * Once called, the interrupt flag will be set
 * for the received message to be processed in
 * the main loop.
 */
ISR(INT0_vect) {
  // Just set the flag
  flags.can_flag = 1;
}

/*
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

/*
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
