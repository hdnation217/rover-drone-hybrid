"""Centralized action names for the hybrid rover/drone stack."""

# One source of truth keeps action names consistent across nodes.
DRIVE_FORWARD = "DRIVE_FORWARD"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
STOP = "STOP"
MOVE_BACKWARD = "MOVE_BACKWARD"
SWITCH_TO_DRONE = "SWITCH_TO_DRONE"
SWITCH_TO_ROVER = "SWITCH_TO_ROVER"

# One source of truth keeps air action names consistent across nodes.
MOVE_FORWARD_AIR = "MOVE_FORWARD_AIR"
MOVE_BACKWARD_AIR = "MOVE_BACKWARD_AIR"
MOVE_LEFT_AIR = "MOVE_LEFT_AIR"
MOVE_RIGHT_AIR = "MOVE_RIGHT_AIR"
ASCEND = "ASCEND"
DESCEND = "DESCEND"
HOVER = "HOVER"
LAND = "LAND"
LANDED = "LANDED"

# One source of truth keeps critical action gating consistent across nodes.
CRITICAL_ACTIONS = {SWITCH_TO_DRONE, SWITCH_TO_ROVER, LAND}

# One source of truth keeps rover action validation consistent across nodes.
ROVER_ACTIONS = {DRIVE_FORWARD, TURN_LEFT, TURN_RIGHT, STOP, MOVE_BACKWARD}

# One source of truth keeps drone action validation consistent across nodes.
DRONE_ACTIONS = {
    MOVE_FORWARD_AIR,
    MOVE_BACKWARD_AIR,
    MOVE_LEFT_AIR,
    MOVE_RIGHT_AIR,
    ASCEND,
    DESCEND,
    HOVER,
    LAND,
    LANDED,
}
