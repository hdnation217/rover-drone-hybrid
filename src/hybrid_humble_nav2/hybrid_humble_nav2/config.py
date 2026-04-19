"""Shared enums and defaults for offline and ROS runtimes."""
from enum import Enum


# An enum keeps autonomy mode values explicit and typo-safe.
class AutonomyMode(str, Enum):
    MANUAL = "MANUAL"
    SEMI_AUTONOMOUS = "SEMI_AUTONOMOUS"
    FULLY_AUTONOMOUS = "FULLY_AUTONOMOUS"


# A default mode lets offline tools run without ROS parameters.
DEFAULT_AUTONOMY_MODE = AutonomyMode.MANUAL

# A default mode lets offline tools start on the ground vehicle.
DEFAULT_VEHICLE_MODE = "ROVER"

# A default topic name keeps ROS wiring easy to change later.
DEFAULT_DETECTIONS_TOPIC = "/detections"

# A default topic name keeps ROS wiring easy to change later.
DEFAULT_SCAN_TOPIC = "/scan"

# A default topic name keeps the filtered obstacle scan discoverable.
DEFAULT_OBSTACLES_TOPIC = "/obstacles"

# A default topic name keeps the high-level decision visible to other nodes.
DEFAULT_ACTION_TOPIC = "/hybrid_action"
