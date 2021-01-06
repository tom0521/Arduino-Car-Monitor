#include <stdint.h>
#include "obd2.h"
#include "mcp2515.h"

/*
 * OBD Request
 * 
 * Sends a can frame to MCP2515. The CAN
 * frame contains an OBD request with the
 * PID being the PID passed as an argument
 */
float obd_read_pid (uint8_t pid) {
  // Create the frame to be sent
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

  // Try to send the frame and return the result
  if (mcp_tx_message(&frame)) {
    while (!mcp_message_waiting())
      ;
    
    if (!mcp_rx_message(&frame)) {
        return -1;
    }
    // Handle the received data
    switch (frame.data[OBD_FRAME_PID])
    {
        case OBD_ENGINE_SPEED:
            return ((256 * frame.data[OBD_FRAME_A]) + frame.data[OBD_FRAME_B]) / 4.f;
        case OBD_VEHICLE_SPEED:
            return frame.data[OBD_FRAME_A] * 0.62137119f;
        case OBD_FUEL_TANK_LEVEL:
            return (100 / 255.f) * frame.data[OBD_FRAME_A];
        case OBD_DIST_CODE_CLR:
            return ((frame.data[OBD_FRAME_A] << 8) + frame.data[OBD_FRAME_B]) * 0.62137119f;
        default:
            return -1;
    }
  } else {
    return -1;
  }
}