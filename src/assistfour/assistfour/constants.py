from math import radians

# Frame names.
ROBOT_FRAME = "base_link"
WORLD_FRAME = "odom"
FEEDER_FRAME = "feeder"
COLUMN_1_FRAME = "column_1"
COLUMN_2_FRAME = "column_2"
COLUMN_3_FRAME = "column_3"
COLUMN_4_FRAME = "column_4"
COLUMN_5_FRAME = "column_5"
COLUMN_6_FRAME = "column_6"
COLUMN_7_FRAME = "column_7"
END_FRAME = "link_grasp_center"

COLUMN_MAP = {
    1: COLUMN_1_FRAME,
    2: COLUMN_2_FRAME,
    3: COLUMN_3_FRAME,
    4: COLUMN_4_FRAME,
    5: COLUMN_5_FRAME,
    6: COLUMN_6_FRAME,
    7: COLUMN_7_FRAME,
}

# Rates.
MAX_TF_AGE = 0.5  # seconds
RECENT_TF_POLL_TIME = 0.25  # second
RECENT_TF_TIMEOUT = 10.0  # seconds
SEARCH_SPIN_RATE = 0.2  # rad / sec; must be <= Pi
MARKER_SEARCH_PERIOD = 0.5  # seconds (2 Hz); must be >= 2 Hz (<= 0.5 seconds)
MAX_FORWARD_SPEED = 0.2  # m / sec

# Minimums
MINIMUM_ANGLE_THRESHOLD = 0.04  # rad

# Offsets.
OFFSET_FROM_MARKER = 0.65
FEEDER_LIFT_OFFSET = 0.1
FEEDER_ARM_OFFSET = 0.06
COLUMN_LIFT_OFFSET = 0.2
COLUMN_ARM_OFFSET = 0.05

# Joint values.
HEAD_SEARCH_TILT = -0.3
LIFT_MID_HEIGHT = 0.45
GRIPPER_CLOSE = -0.05
GRIPPER_OPEN = 0.3
WRIST_UP = 0.0
WRIST_DOWN = radians(-90)


# Exceptions.
class CancelGoalException(Exception):
    pass
