#include <stdint.h>
#include "obd2.h"
#include "mcp2515.h"

/*
 * Initialize OBD-II
 *
 */
bool obd2_init() {
    return mcp_init(0x01);
}

/*
 * OBD2 Request
 * 
 * Sends a can frame to MCP2515. The CAN
 * frame contains an OBD2 request with the
 * PID being the PID passed as an argument
 */
obd2_value obd2_read_pid (uint8_t pid) {
  mcp_can_frame frame;
  obd2_value retval;
  
  // Create the frame to be sent
  frame.sid = CAN_ECU_REQ;
  frame.srr = 0;
  frame.ide = 0;
  frame.eid = 0;
  frame.rtr = 0;
  frame.dlc = 8;
  frame.data[OBD2_FRAME_LENGTH] = 0x2;
  frame.data[OBD2_FRAME_MODE] = OBD2_CUR_DATA;
  frame.data[OBD2_FRAME_PID] = pid;

  // Try to send the frame and return the result
  if (!mcp_tx_message(&frame)) {
    retval.f = -1;
    return retval;
  }

  while (!mcp_message_waiting())
    ;

  if (!mcp_rx_message(&frame)) {
    retval.f = -1;
    return retval;
  }

  // Handle the received data
  switch (frame.data[OBD2_FRAME_PID]) {
    case OBD2_PID_SUPPORT_1:
    case OBD2_PID_SUPPORT_2:
    case OBD2_PID_SUPPORT_3:
      retval.u32 = frame.data[OBD2_FRAME_A] << 24 +
                   frame.data[OBD2_FRAME_B] << 16 +
                   frame.data[OBD2_FRAME_C] << 8  +
                   frame.data[OBD2_FRAME_D]; 
      break;
    case OBD2_ENGINE_SPEED:
      retval.f = ((256 * frame.data[OBD2_FRAME_A]) + 
                      frame.data[OBD2_FRAME_B]) / 4.f;
      break;
    case OBD2_VEHICLE_SPEED:
      retval.f = frame.data[OBD2_FRAME_A] * 0.62137119f;
      break;
    case OBD2_FUEL_TANK_LEVEL:
      retval.f = (100 / 255.f) * frame.data[OBD2_FRAME_A];
      break;
    case OBD2_DIST_CODE_CLR:
      retval.f = ((frame.data[OBD2_FRAME_A] << 8) + 
                      frame.data[OBD2_FRAME_B]) * 0.62137119f;
      break;
    case OBD2_ENGINE_FUEL_RATE:
      retval.f = ((256 * frame.data[OBD2_FRAME_A]) + 
                      frame.data[OBD2_FRAME_B]) / 20.f;
      break;
    default: 
      retval.f = -1;
      break;
  }
  return retval;
}
