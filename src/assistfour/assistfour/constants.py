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

# Rates.
SEARCH_SPIN_RATE = 0.5  # rad / sec; must be <= Pi
MARKER_SEARCH_PERIOD = 0.5  # seconds (2 Hz); must be >= 2 Hz (<= 0.5 seconds)
MAX_FORWARD_SPEED = 0.5  # m / sec
MINIMUM_FORWARD_DISTANCE_THRESHOLD = 0.05  # m

# Minimums
MINIMUM_ANGLE_THRESHOLD = 0.05

COLUMN_MAP = {
    1: COLUMN_1_FRAME,
    2: COLUMN_2_FRAME,
    3: COLUMN_3_FRAME,
    4: COLUMN_4_FRAME,
    5: COLUMN_5_FRAME,
    6: COLUMN_6_FRAME,
    7: COLUMN_7_FRAME,
}
