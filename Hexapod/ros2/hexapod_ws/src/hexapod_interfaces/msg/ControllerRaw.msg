# ControllerRaw.msg
# Header with timestamp
builtin_interfaces/Time stamp

# Controller axes (normalized -1.0 to 1.0)
# Standard mapping: [left_x, left_y, right_x, right_y, ...]
# Can be extended for additional axes (triggers, etc.)
float32[] axes

# Discrete command (see Command enum below)
uint16 command_id

# Optional parameters for commands
# e.g., [gait_id], [leg_id, joint_id, offset], [height_mm]
float32[] arguments

# Input source identifier
uint8 source_id

# Source ID constants
uint8 SOURCE_ESPNOW = 0      # Physical ESP-NOW controller
uint8 SOURCE_VIRTUAL_JOY = 1 # Virtual joystick (GUI)
uint8 SOURCE_KEYBOARD = 2    # Keyboard teleop
uint8 SOURCE_GAMEPAD = 3     # USB/Bluetooth gamepad
uint8 SOURCE_AUTONOMY = 4    # Autonomous navigation stack

# Command ID constants (matches embedded Command enum)
uint16 CMD_NONE = 0
uint16 CMD_SET_GAIT = 1
uint16 CMD_SET_MODE = 2
uint16 CMD_SET_CONFIG = 3
uint16 CMD_HOME_STANCE = 4
uint16 CMD_REQUEST_CONFIG = 5
uint16 CMD_SET_HEIGHT = 6
uint16 CMD_EMERGENCY_STOP = 7