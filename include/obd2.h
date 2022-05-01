#ifndef OBD2_H
#define OBD2_H

#include <stdbool.h>

/* OBD-II Modes */
#define OBD2_CUR_DATA    0x01    // Get the current data
#define OBD2_FREEZE      0x02    // Freeze-frame data
#define OBD2_CODES       0x03
#define OBD2_CLEAR_CODES 0x04
#define OBD2_TEST_O2     0x05
#define OBD2_TEST_ETC    0x06
#define OBD2_CUR_CODES   0x07
#define OBD2_ONBOARD     0x08
#define OBD2_VEH_INFO    0x09
#define OBD2_PERM_CODES  0x0A

/* OBD-II PIDs */
#define OBD2_PID_SUPPORT_1       0x00    // Supported OBD-II PIDs
#define OBD2_SINCE_DTC_CLEARED   0x01    
#define OBD2_FREEZE_DTC          0x02
#define OBD2_FUEL_SYS_STATUS     0x03
#define OBD2_ENGINE_LOAD         0x04
#define OBD2_COOLANT_TEMP        0x05
#define OBD2_SHORT_FUEL_TRIM_1   0x06
#define OBD2_LONG_FUEL_TRIM_1    0x07
#define OBD2_SHORT_FUEL_TRIM_2   0x08
#define OBD2_LONG_FUEL_TRIM_2    0x09
#define OBD2_FUEL_PRESSURE       0x0A
#define OBD2_INTAKE_PRESSURE     0x0B
#define OBD2_ENGINE_SPEED        0x0C
#define OBD2_VEHICLE_SPEED       0x0D
#define OBD2_TIMING_ADVANCE      0x0E
#define OBD2_INTAKE_TEMP         0x0F
#define OBD2_MAF_RATE            0x10
#define OBD2_THROTTLE_POS        0x11
#define OBD2_2ND_AIR_STATUS      0x12
#define OBD2_O2_2_BANKS          0x13
#define OBD2_O2_2_BANKS_1        0x14
#define OBD2_O2_2_BANKS_2        0x15
#define OBD2_O2_2_BANKS_3        0x16
#define OBD2_O2_2_BANKS_4        0x17
#define OBD2_O2_2_BANKS_5        0x18
#define OBD2_O2_2_BANKS_6        0x19
#define OBD2_O2_2_BANKS_7        0x1A
#define OBD2_O2_2_BANKS_8        0x1B
#define OBD2_STANDARDS           0x1C
#define OBD2_O2_4_BANKS          0x1D
#define OBD2_AUX_INPUT_STATUS    0x1E
#define OBD2_RUN_TIME            0x1F
#define OBD2_PID_SUPPORT_2       0x20
#define OBD2_DIST_WITH_MIL       0x21
#define OBD2_FUEL_RAIL_PRESSURE  0x22
#define OBD2_FUEL_RAIL_GAUGE     0x23

#define OBD2_FUEL_TANK_LEVEL     0x2F

#define OBD2_DIST_CODE_CLR       0x31

#define OBD2_PID_SUPPORT_3       0x40

#define OBD2_ENGINE_FUEL_RATE    0x5E

#define OBD2_ODOMETER            0xA6

/* The OBD-II Data Frame */
#define OBD2_FRAME_LENGTH      0   // OBD-II data length
#define OBD2_FRAME_MODE        1   // OBD-II data mode
#define OBD2_FRAME_PID         2   // OBD-II PID
#define OBD2_FRAME_A           3   // OBD-II data A
#define OBD2_FRAME_B           4   // OBD-II data B
#define OBD2_FRAME_C           5   // OBD-II data C
#define OBD2_FRAME_D           6   // OBD-II data D

/* * * * * * * * * * * * * * * */
/*                             */
/*    Function Definitions     */
/*                             */
/* * * * * * * * * * * * * * * */
bool obd2_init ();
float obd2_read_pid (uint8_t pid);

#endif // OBD2_H
