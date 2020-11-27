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

#define CURSOR LCD_R_ARROW

/* flags needed for this project */
volatile struct flags
{
  uint8_t rgb : 3;        // RGB LED color
  uint8_t cursor : 2;
} flags;

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
  // Falling edge of INT1 generates interrupt
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
  // Rising edge of INT0 generates interrupt
  EICRA |= (1 << ISC01);
  EICRA |= (1 << ISC00);
  // Enable interrupts for INT0
  EIMSK |= (1 << INT0);

  // Initialize CAN-BUS communication
  if (mcp_init(0x01)) {
    // Create the catalyst request
    struct process * p = (struct process *) malloc(sizeof(*p));
    p->func = obd_request;
    p->arg = OBD_ENGINE_SPEED;
    list_enqueue(&queue, &p->elem);
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
    if (p->func(p->arg) == 0) { // ... and free it on success
      free(p);
    } else {  // Push it to the back of the line 
      /** TODO: Good for now */
      list_enqueue(&queue, &p->elem);
    }
  } else {
    // Idle state
  }
}

/* * * * * * * * * * * * * * * * * * * * * */
/*          CAN Request Functions          */
/* * * * * * * * * * * * * * * * * * * * * */
/*
 * OBD Request
 * 
 * Sends a can frame to MCP2515. The CAN
 * frame contains an OBD request with the
 * PID being the PID passed as an argument
 */
int8_t obd_request (uint8_t pid) {
  mcp_can_frame frame;
  frame.sid = CAN_ECU_REQ;
  frame.srr = 0;
  frame.ide = 0;
  frame.eid = 0;
  frame.rtr = 0;
  frame.dlc = 8;
  frame.data[OBD_FRAME_LENGTH] = 0x2;
  frame.data[OBD_FRAME_MODE] = OBD_CUR_DATA;
  frame.data[OBD_FRAME_PID] = pid;

  if (!mcp_tx_message(&frame)) {
    return -1;
  } else {
    return 0;
  }
}

/*
 * OBD Response
 * 
 * Retrives the CAN frame from the
 * MCP2515. This frame contains an
 * OBD-II response.
 */
int8_t obd_response (uint8_t aux) {
  mcp_can_frame frame;

  if (!mcp_rx_message (&frame)) {
    return -1;
  }

  switch (frame.data[OBD_FRAME_PID])
  {
  case OBD_ENGINE_SPEED: {
    float engine_speed = ((256 * frame.data[OBD_FRAME_A]) + frame.data[OBD_FRAME_B]) / 4.f;
    Serial.println(engine_speed);
    break;
  }
  default:
    // Unimplemented PID
    Serial.println("Unimplemented PID");
    // Don't want to re-queue an unimplemented pid
    return 0;
    break;
  }

  // Re-add the pid to the job queue
  struct process * p = (struct process *) malloc(sizeof(*p));
  p->func = obd_request;
  p->arg = frame.data[OBD_FRAME_PID];
  list_enqueue (&queue, &p->elem);

  return 0;
}

// Print the encoder to the screen
int8_t print_encoder (uint8_t saved_enc) {
  // lcd_print (encoder);
  return 0;
}

int8_t update_cursor (uint8_t prev_enc) {
  uint8_t row = prev_enc % 4;
  lcd_set_cursor(LCD_ROW(row));
  lcd_putc(' ');

  row = encoder % 4;
  lcd_set_cursor(LCD_ROW(row));
  lcd_putc(CURSOR);

  return 0;
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
ISR(INT0_vect) {
  // Add a get message process to the job queue
  struct process * p = (struct process *) malloc(sizeof(*p));
  p->func = obd_response;
  list_enqueue(&queue, &p->elem);
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
  // Add a select handler to the queue
  encoder = 0;
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

  if (rotary_state == 0x9) {
    ++encoder;
  }
  else if (rotary_state == 0x3) {
    --encoder;
  }
}
