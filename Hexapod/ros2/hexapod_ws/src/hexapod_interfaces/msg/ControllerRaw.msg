# ControllerRaw.msg
float32[] axes       # e.g., [left_x, left_y, right_x, right_y]
uint16 command_id    # enum for discrete actions (change gait, set height, etc.)
float32[] arguments  # optional parameters for commands
uint8 source_id      # which controller
builtin_interfaces/Time stamp