#ifndef INC_CONFIG_COMMANDS_H_
#define INC_CONFIG_COMMANDS_H_

// Configuration Commands
#define CMD_INITIALIZE     0x01    // Initialize device
#define CMD_SET_CENTER     0x02    // Set center position
#define CMD_SET_POWER      0x03    // Set power level
#define CMD_SET_SPRING     0x04    // Set spring strength
#define CMD_SET_MAX_ANGLE  0x05    // Set maximum rotation angle

// Response Status
#define STATUS_OK          0x00    // Command executed successfully
#define STATUS_ERROR       0x01    // Command failed

// Report IDs
#define REPORT_ID_WHEEL    0x01    // Wheel position report
#define REPORT_ID_CONFIG   0x14    // Configuration command report (20)
#define REPORT_ID_RESPONSE 0x15    // Configuration response report

#endif // INC_CONFIG_COMMANDS_H_ 