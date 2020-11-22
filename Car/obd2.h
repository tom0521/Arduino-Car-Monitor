#ifndef OBD2_H
#define OBD2_H

/* OBD-II Modes */
#define OBD_CUR_DATA    0x01    // Get the current data
#define OBD_FREEZE      0x02    // Freeze-frame data
#define OBD_CODES       0x03
#define OBD_CLEAR_CODES 0x04
#define OBD_TEST_O2     0x05
#define OBD_TEST_ETC    0x06
#define OBD_CUR_CODES   0x07
#define OBD_ONBOARD     0x08
#define OBD_VEH_INFO    0x09
#define OBD_PERM_CODES  0x0A

/* OBD-II PIDs */
#define OBD_PID_SUPPORT_1       0x00    // Supported OBD-II PIDs
#define OBD_SINCE_DTC_CLEARED   0x01    
#define OBD_FREEZE_DTC          0x02
#define OBD_FUEL_SYS_STATUS     0x03
#define OBD_ENGINE_LOAD         0x04
#define OBD_COOLANT_TEMP        0x05
#define OBD_SHORT_FUEL_TRIM_1   0x06
#define OBD_LONG_FUEL_TRIM_1    0x07
#define OBD_SHORT_FUEL_TRIM_2   0x08
#define OBD_LONG_FUEL_TRIM_2    0x09
#define OBD_FUEL_PRESSURE       0x0A
#define OBD_INTAKE_PRESSURE     0x0B
#define OBD_ENGINE_SPEED        0x0C
#define OBD_VEHICLE_SPEED       0x0D
#define OBD_TIMING_ADVANCE      0x0E
#define OBD_INTAKE_TEMP         0x0F
#define OBD_MAF_RATE            0x10
#define OBD_THROTTLE_POS        0x11
#define OBD_2ND_AIR_STATUS      0x12
#define OBD_O2_2_BANKS          0x13
#define OBD_O2_2_BANKS_1        0x14
#define OBD_O2_2_BANKS_2        0x15
#define OBD_O2_2_BANKS_3        0x16
#define OBD_O2_2_BANKS_4        0x17
#define OBD_O2_2_BANKS_5        0x18
#define OBD_O2_2_BANKS_6        0x19
#define OBD_O2_2_BANKS_7        0x1A
#define OBD_O2_2_BANKS_8        0x1B
#define OBD_STANDARDS           0x1C
#define OBD_O2_4_BANKS          0x1D
#define OBD_AUX_INPUT_STATUS    0x1E
#define OBD_RUN_TIME            0x1F
#define OBD_PID_SUPPORT_2       0x20
#define OBD_DIST_WITH_MIL       0x21
#define OBD_FUEL_RAIL_PRESSURE  0x22
#define OBD_FUEL_RAIL_GAUGE     0x23

#define OBD_PID_SUPPORT_3       0x40

/* The OBD-II Data Frame */
#define OBD_FRAME_LENGTH      0   // OBD-II data length
#define OBD_FRAME_MODE        1   // OBD-II data mode
#define OBD_FRAME_PID         2   // OBD-II PID
#define OBD_FRAME_A           3   // OBD-II data A
#define OBD_FRAME_B           4   // OBD-II data B
#define OBD_FRAME_C           5   // OBD-II data C
#define OBD_FRAME_D           6   // OBD-II data D

#endif // OBD2_H