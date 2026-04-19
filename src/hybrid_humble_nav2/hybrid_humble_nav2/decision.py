"""Core rover/drone decision logic with cost-aware mode switching."""
from typing import Any, Dict, Tuple

from .actions import (
    ASCEND,
    DRIVE_FORWARD,
    HOVER,
    LAND,
    MOVE_BACKWARD,
    MOVE_FORWARD_AIR,
    MOVE_LEFT_AIR,
    MOVE_RIGHT_AIR,
    STOP,
    SWITCH_TO_DRONE,
    SWITCH_TO_ROVER,
    TURN_LEFT,
    TURN_RIGHT,
)


# This value controls how long a rover waits before backing up and escalating.
ROVER_STATIC_FRAMES_FOR_BACKUP = 3

# This value controls how many clear frames are needed before landing.
DRONE_CLEAR_FRAMES_FOR_LAND = 2

# This margin avoids switching modes when rover and drone scores are nearly tied.
MODE_SWITCH_MARGIN = 1.0


# This helper formats a readable obstacle description for debugging and logs.
def _obstacle_description(tracking_info: Dict[str, Any]) -> str:
    obstacle = tracking_info.get("main_obstacle")
    if obstacle is None:
        return "No main obstacle"

    label = obstacle.get("label", "unknown")
    confidence = obstacle.get("confidence", 0.0)
    position = tracking_info.get("obstacle_position", "unknown")
    history_length = tracking_info.get("history_length", 0)

    if tracking_info.get("is_moving", False):
        motion = "moving"
    elif tracking_info.get("is_static", False):
        motion = f"static for {history_length} frame(s)"
    else:
        motion = "newly observed"

    return f"{motion} {label} at {position} (confidence={confidence:.2f})"


# This helper builds a standardized result payload for all decisions.
def make_result(
    current_mode: str,
    current_action: str,
    reason: str,
    requested_mode: str | None = None,
    transition_state: str | None = None,
    confirmation_required: bool = False,
    confirmed: bool = False,
    decision_scores: Dict[str, float] | None = None,
) -> Dict[str, Any]:
    return {
        "current_mode": current_mode,
        "current_action": current_action,
        "requested_mode": requested_mode,
        "transition_state": transition_state,
        "reason": reason,
        "confirmation_required": confirmation_required,
        "confirmed": confirmed,
        "decision_scores": decision_scores or {},
    }


# This helper turns the current state into rover-vs-drone cost scores.
def _compute_mode_scores(
    tracking_info: Dict[str, Any],
    terrain_info: Dict[str, Any],
) -> Dict[str, float]:
    obstacle = tracking_info.get("main_obstacle") or {}
    area_ratio = float(tracking_info.get("obstacle_area_ratio", 0.0))
    blocking = 1.0 if tracking_info.get("blocking_forward_path", False) else 0.0
    scan_blocked = 1.0 if tracking_info.get("scan_blocked", False) else 0.0
    route_valid = 1.0 if tracking_info.get("route_valid", True) else 0.0
    is_static = 1.0 if tracking_info.get("is_static", False) else 0.0
    is_moving = 1.0 if tracking_info.get("is_moving", False) else 0.0
    history_length = float(tracking_info.get("history_length", 0))
    terrain_warning = 1.0 if terrain_info.get("terrain_warning", False) else 0.0
    landing_safe = 1.0 if terrain_info.get("landing_safe", False) else 0.0
    confidence = float(obstacle.get("confidence", 0.0))

    persistent_blockage = blocking * min(history_length / 3.0, 1.0)
    obstacle_scale = min(area_ratio / 0.12, 1.0)

    rover_score = (
        1.0
        + blocking * 3.0
        + scan_blocked * 2.0
        + (1.0 - route_valid) * 4.0
        + terrain_warning * 4.0
        + persistent_blockage * 2.5
        + obstacle_scale * 2.0
        + is_static * 1.0
        + confidence * 0.5
    )

    drone_score = (
        3.0
        + is_moving * 1.2
        + obstacle_scale * 0.4
        + blocking * 0.5
        - terrain_warning * 2.0
        - landing_safe * 0.5
        - (1.0 - route_valid) * 0.8
    )

    return {
        "rover": round(rover_score, 2),
        "drone": round(drone_score, 2),
    }


# This helper picks the safer ground maneuver when staying in rover mode.
def _choose_rover_avoidance_action(tracking_info: Dict[str, Any]) -> Tuple[str, str]:
    position = tracking_info.get("obstacle_position")
    if position == "left":
        return TURN_RIGHT, "Staying in rover mode and steering right around the obstacle."
    if position == "right":
        return TURN_LEFT, "Staying in rover mode and steering left around the obstacle."
    return MOVE_BACKWARD, "Staying in rover mode and backing up to create space for replanning."


# This function chooses the next rover action from normalized state.
def decide_rover_action(
    tracking_info: Dict[str, Any],
    terrain_info: Dict[str, Any],
    transition_state: str | None,
    confirmed_for_drone: bool,
) -> Dict[str, Any]:
    has_obstacle = tracking_info.get("has_obstacle", False)
    position = tracking_info.get("obstacle_position")
    blocking_forward_path = tracking_info.get("blocking_forward_path", False)
    is_moving = tracking_info.get("is_moving", False)
    is_static = tracking_info.get("is_static", False)
    history_length = tracking_info.get("history_length", 0)
    route_valid = tracking_info.get("route_valid", True)
    goal_reached = tracking_info.get("goal_reached", False)
    scan_blocked = tracking_info.get("scan_blocked", False)

    terrain_warning = terrain_info.get("terrain_warning", False)
    terrain_type = terrain_info.get("terrain_type", "unknown")
    obstacle_text = _obstacle_description(tracking_info)
    decision_scores = _compute_mode_scores(tracking_info, terrain_info)

    # A completed goal should halt the rover instead of continuing forward.
    if goal_reached:
        return make_result(
            current_mode="ROVER",
            current_action=STOP,
            reason="Goal has been reached. Holding position.",
            decision_scores=decision_scores,
        )

    # A pending switch keeps the rover paused until the operator approves takeoff.
    if transition_state == "TO_DRONE_PENDING":
        if confirmed_for_drone:
            return make_result(
                current_mode="ROVER",
                current_action=SWITCH_TO_DRONE,
                requested_mode="DRONE",
                transition_state="TO_DRONE_ACTIVE",
                reason=(
                    "Drone mode has the lower risk/cost score "
                    f"(rover={decision_scores['rover']}, drone={decision_scores['drone']}). "
                    "Human confirmation received. Switching from rover mode to drone mode."
                ),
                confirmation_required=False,
                confirmed=True,
                decision_scores=decision_scores,
            )
        return make_result(
            current_mode="ROVER",
            current_action=STOP,
            requested_mode="DRONE",
            transition_state="TO_DRONE_PENDING",
            reason=(
                "Drone mode has the lower risk/cost score "
                f"(rover={decision_scores['rover']}, drone={decision_scores['drone']}). "
                "Waiting for human confirmation before takeoff."
            ),
            confirmation_required=True,
            confirmed=False,
            decision_scores=decision_scores,
        )

    # Dangerous terrain should trigger a rover-to-drone escalation.
    if terrain_warning:
        return make_result(
            current_mode="ROVER",
            current_action=STOP,
            requested_mode="DRONE",
            transition_state="TO_DRONE_PENDING",
            reason=(
                f"Dangerous terrain '{terrain_type}' detected ahead. "
                f"Drone travel is safer here (rover={decision_scores['rover']}, drone={decision_scores['drone']})."
            ),
            confirmation_required=True,
            confirmed=False,
            decision_scores=decision_scores,
        )

    # An invalid ground route should escalate toward drone mode.
    if not route_valid:
        return make_result(
            current_mode="ROVER",
            current_action=STOP,
            requested_mode="DRONE",
            transition_state="TO_DRONE_PENDING",
            reason=(
                "Nav2 reports that the rover route is no longer valid. "
                f"Switching to drone mode is preferred (rover={decision_scores['rover']}, drone={decision_scores['drone']})."
            ),
            confirmation_required=True,
            confirmed=False,
            decision_scores=decision_scores,
        )

    # A clear route with no obstacle lets the rover continue normally.
    if not has_obstacle and not blocking_forward_path and not scan_blocked:
        return make_result(
            current_mode="ROVER",
            current_action=DRIVE_FORWARD,
            requested_mode=None,
            transition_state=None,
            reason="No major obstacle detected and the rover route remains valid.",
            decision_scores=decision_scores,
        )

    # A non-blocking obstacle on the left suggests turning right.
    if position == "left" and not blocking_forward_path:
        return make_result(
            current_mode="ROVER",
            current_action=TURN_RIGHT,
            reason=f"Obstacle detected on the left. {obstacle_text}.",
            decision_scores=decision_scores,
        )

    # A non-blocking obstacle on the right suggests turning left.
    if position == "right" and not blocking_forward_path:
        return make_result(
            current_mode="ROVER",
            current_action=TURN_LEFT,
            reason=f"Obstacle detected on the right. {obstacle_text}.",
            decision_scores=decision_scores,
        )

    # A blocking obstacle in the path requires either waiting, avoiding, or escalating.
    if blocking_forward_path or scan_blocked:
        if is_moving:
            return make_result(
                current_mode="ROVER",
                current_action=STOP,
                reason=(
                    "Obstacle is moving in the forward path. "
                    f"Holding position until the scene stabilizes. {obstacle_text}."
                ),
                decision_scores=decision_scores,
            )

        prefer_drone = decision_scores["drone"] + MODE_SWITCH_MARGIN < decision_scores["rover"]

        if is_static and prefer_drone:
            if history_length >= ROVER_STATIC_FRAMES_FOR_BACKUP:
                return make_result(
                    current_mode="ROVER",
                    current_action=MOVE_BACKWARD,
                    requested_mode="DRONE",
                    transition_state="TO_DRONE_PENDING",
                    reason=(
                        "Persistent forward blockage makes drone mode the safer lower-cost option. "
                        f"Backing up before takeoff. Scores: rover={decision_scores['rover']}, drone={decision_scores['drone']}. "
                        f"{obstacle_text}."
                    ),
                    confirmation_required=True,
                    confirmed=False,
                    decision_scores=decision_scores,
                )
            return make_result(
                current_mode="ROVER",
                current_action=STOP,
                requested_mode="DRONE",
                transition_state="TO_DRONE_PENDING",
                reason=(
                    "Obstacle is persistent and blocks the rover path. Drone mode now looks safer than ground motion. "
                    f"Scores: rover={decision_scores['rover']}, drone={decision_scores['drone']}."
                ),
                confirmation_required=True,
                confirmed=False,
                decision_scores=decision_scores,
            )

        if history_length < 2:
            return make_result(
                current_mode="ROVER",
                current_action=STOP,
                reason=(
                    "Obstacle detected in the forward path. Waiting for one more frame before committing "
                    f"to avoidance or takeoff. {obstacle_text}."
                ),
                decision_scores=decision_scores,
            )

        avoid_action, avoid_reason = _choose_rover_avoidance_action(tracking_info)
        return make_result(
            current_mode="ROVER",
            current_action=avoid_action,
            reason=(
                f"{avoid_reason} Ground motion still looks competitive versus flying "
                f"(rover={decision_scores['rover']}, drone={decision_scores['drone']}). {obstacle_text}."
            ),
            decision_scores=decision_scores,
        )

    # The default rover behavior is to continue forward if nothing critical is wrong.
    return make_result(
        current_mode="ROVER",
        current_action=DRIVE_FORWARD,
        reason=f"Obstacle detected but not blocking the rover path. {obstacle_text}.",
        decision_scores=decision_scores,
    )


# This function chooses the next drone action from normalized state.
def decide_drone_action(
    tracking_info: Dict[str, Any],
    terrain_info: Dict[str, Any],
    transition_state: str | None,
    clear_landing_streak: int,
    confirmed_for_landing: bool,
    confirmed_for_rover: bool,
) -> Dict[str, Any]:
    has_obstacle = tracking_info.get("has_obstacle", False)
    position = tracking_info.get("obstacle_position")
    blocking_forward_path = tracking_info.get("blocking_forward_path", False)
    goal_reached = tracking_info.get("goal_reached", False)
    obstacle_text = _obstacle_description(tracking_info)
    decision_scores = _compute_mode_scores(tracking_info, terrain_info)

    landing_safe = terrain_info.get("landing_safe", False)
    terrain_type = terrain_info.get("terrain_type", "unknown")

    # A completed landing keeps the drone hovering until rover handoff is approved.
    if transition_state == "TO_ROVER_PENDING":
        if confirmed_for_rover:
            return make_result(
                current_mode="DRONE",
                current_action=SWITCH_TO_ROVER,
                requested_mode="ROVER",
                transition_state="TO_ROVER_ACTIVE",
                reason=(
                    "Landing sequence completed and ground mode is now preferred again. "
                    "Human confirmation received. Switching from drone mode back to rover mode."
                ),
                confirmation_required=False,
                confirmed=True,
                decision_scores=decision_scores,
            )
        return make_result(
            current_mode="DRONE",
            current_action=HOVER,
            requested_mode="ROVER",
            transition_state="TO_ROVER_PENDING",
            reason="Landing sequence completed. Waiting for human confirmation before switching back to rover mode.",
            confirmation_required=True,
            confirmed=False,
            decision_scores=decision_scores,
        )

    # A completed goal should transition the drone toward landing logic.
    if goal_reached and landing_safe and clear_landing_streak >= DRONE_CLEAR_FRAMES_FOR_LAND:
        if confirmed_for_landing:
            return make_result(
                current_mode="DRONE",
                current_action=LAND,
                requested_mode="ROVER",
                transition_state="TO_ROVER_PENDING",
                reason="Goal has been reached and the landing zone is safe. Landing approval received.",
                confirmed=True,
                decision_scores=decision_scores,
            )

    # A blocking obstacle in front of the drone should trigger a climb.
    if has_obstacle and blocking_forward_path:
        return make_result(
            current_mode="DRONE",
            current_action=ASCEND,
            reason=f"Obstacle detected ahead in air path. Ascending to avoid it. {obstacle_text}.",
            decision_scores=decision_scores,
        )

    # A side obstacle on the left suggests moving right in the air.
    if has_obstacle and position == "left":
        return make_result(
            current_mode="DRONE",
            current_action=MOVE_RIGHT_AIR,
            reason=f"Obstacle detected on the left in drone mode. {obstacle_text}.",
            decision_scores=decision_scores,
        )

    # A side obstacle on the right suggests moving left in the air.
    if has_obstacle and position == "right":
        return make_result(
            current_mode="DRONE",
            current_action=MOVE_LEFT_AIR,
            reason=f"Obstacle detected on the right in drone mode. {obstacle_text}.",
            decision_scores=decision_scores,
        )

    # A clear and safe landing zone starts or continues landing confirmation.
    if not has_obstacle and landing_safe:
        if clear_landing_streak < DRONE_CLEAR_FRAMES_FOR_LAND:
            return make_result(
                current_mode="DRONE",
                current_action=HOVER,
                reason=f"Air path is clear and terrain '{terrain_type}' is safe for landing. Holding steady to confirm landing zone.",
                decision_scores=decision_scores,
            )

        if confirmed_for_landing:
            return make_result(
                current_mode="DRONE",
                current_action=LAND,
                requested_mode="ROVER",
                transition_state="TO_ROVER_PENDING",
                reason=(
                    f"Landing zone confirmed safe for {clear_landing_streak} clear frame(s). "
                    "Landing approval received. Beginning landing sequence."
                ),
                confirmed=True,
                decision_scores=decision_scores,
            )

        return make_result(
            current_mode="DRONE",
            current_action=HOVER,
            requested_mode="ROVER",
            transition_state=None,
            reason=(
                f"Landing zone confirmed safe for {clear_landing_streak} clear frame(s). "
                "Waiting for landing approval."
            ),
            confirmation_required=True,
            confirmed=False,
            decision_scores=decision_scores,
        )

    # The default drone behavior is to continue forward flight.
    return make_result(
        current_mode="DRONE",
        current_action=MOVE_FORWARD_AIR,
        reason="Air path is clear enough to continue forward flight.",
        decision_scores=decision_scores,
    )


# This function dispatches to the correct vehicle-specific decision routine.
def decide_action(
    current_mode: str,
    tracking_info: Dict[str, Any],
    terrain_info: Dict[str, Any],
    transition_state: str | None,
    clear_landing_streak: int = 0,
    confirmed_for_drone: bool = False,
    confirmed_for_landing: bool = False,
    confirmed_for_rover: bool = False,
) -> Dict[str, Any]:
    mode = current_mode.upper()
    if mode == "ROVER":
        return decide_rover_action(
            tracking_info=tracking_info,
            terrain_info=terrain_info,
            transition_state=transition_state,
            confirmed_for_drone=confirmed_for_drone,
        )
    if mode == "DRONE":
        return decide_drone_action(
            tracking_info=tracking_info,
            terrain_info=terrain_info,
            transition_state=transition_state,
            clear_landing_streak=clear_landing_streak,
            confirmed_for_landing=confirmed_for_landing,
            confirmed_for_rover=confirmed_for_rover,
        )
    raise ValueError(f"Unsupported mode: {current_mode}")
