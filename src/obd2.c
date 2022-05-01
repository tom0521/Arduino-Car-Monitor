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
float obd2_read_pid (uint8_t pid) {
  // Create the frame to be sent
  mcp_can_frame frame;
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
  if (!mcp_tx_message(&frame))
    return -1;

  while (!mcp_message_waiting())
    ;

  if (!mcp_rx_message(&frame))
    return -1;

  // Handle the received data
  switch (frame.data[OBD2_FRAME_PID])
  {
    case OBD2_ENGINE_SPEED:
      return ((256 * frame.data[OBD2_FRAME_A]) + 
                frame.data[OBD2_FRAME_B]) / 4.f;
    case OBD2_VEHICLE_SPEED:
      return frame.data[OBD2_FRAME_A] * 0.62137119f;
    case OBD2_FUEL_TANK_LEVEL:
      return (100 / 255.f) * frame.data[OBD2_FRAME_A];
    case OBD2_DIST_CODE_CLR:
      return ((frame.data[OBD2_FRAME_A] << 8) + 
                frame.data[OBD2_FRAME_B]) * 0.62137119f;
    case OBD2_ENGINE_FUEL_RATE:
      return ((256 * frame.data[OBD2_FRAME_A]) + 
                frame.data[OBD2_FRAME_B]) / 20.f;
    default:
      return -1;
  }
}
