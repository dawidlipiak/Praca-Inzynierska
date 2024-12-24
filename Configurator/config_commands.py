# Configuration Commands
CMD_INITIALIZE = 0x01    # Initialize device
CMD_SET_CENTER = 0x02    # Set center position
CMD_SET_POWER = 0x03     # Set power level
CMD_SET_SPRING = 0x04    # Set spring strength
CMD_SET_MAX_ANGLE = 0x05 # Set maximum rotation angle

# Response Status
STATUS_OK = 0x00         # Command executed successfully
STATUS_ERROR = 0x01      # Command failed

# Report IDs
REPORT_ID_WHEEL = 0x01   # Wheel position report
REPORT_ID_CONFIG = 0x14  # Configuration command report (20)
REPORT_ID_RESPONSE = 0x15  # Configuration response report 