"""Shared engine step used by offline and ROS runtimes."""
from typing import Any, Dict

from .config import AutonomyMode
from .controller import apply_autonomy_mode
from .decision import decide_action
from .terrain import get_terrain_warning
from .tracker import ObstacleTracker


# This helper tracks how long a landing zone has remained clear and safe.
def update_clear_landing_streak(
    current_mode: str,
    tracking_info: Dict[str, Any],
    terrain_info: Dict[str, Any],
    existing_streak: int,
) -> int:
    if current_mode != "DRONE":
        return 0

    if not tracking_info.get("has_obstacle", False) and terrain_info.get("landing_safe", False):
        return existing_streak + 1

    return 0


# This helper expands user controls with autonomous approvals when allowed.
def build_effective_controls(
    autonomy_mode: AutonomyMode,
    controls: Dict[str, Any],
) -> Dict[str, Any]:
    effective = dict(controls)
    if autonomy_mode == AutonomyMode.FULLY_AUTONOMOUS:
        effective["approve_drone_switch"] = True
        effective["approve_landing"] = True
        effective["approve_rover_switch"] = True
    return effective


# This helper updates vehicle mode after the gated action is applied.
def apply_mode_transition(
    current_mode: str,
    transition_state: str | None,
    result: Dict[str, Any],
) -> tuple[str, str | None]:
    action = result["current_action"]
    requested_mode = result.get("requested_mode")
    next_transition_state = result.get("transition_state")

    # A blocked action should preserve pending mode requests instead of losing them.
    if not result.get("execution_allowed", True):
        if next_transition_state is not None:
            return current_mode, next_transition_state
        return current_mode, transition_state

    # A confirmed takeoff changes the active vehicle to the drone.
    if action == "SWITCH_TO_DRONE" and requested_mode == "DRONE":
        return "DRONE", None

    # A landing keeps the active vehicle as drone until rover switch is approved.
    if action == "LAND" and requested_mode == "ROVER":
        return "DRONE", "TO_ROVER_PENDING"

    # A confirmed rover handoff changes the active vehicle back to rover.
    if action == "SWITCH_TO_ROVER" and requested_mode == "ROVER":
        return "ROVER", None

    # Most actions leave the active mode unchanged.
    return current_mode, next_transition_state


# This engine owns persistent state across frames and ROS callbacks.
class HybridDecisionEngine:
    def __init__(self, autonomy_mode: AutonomyMode) -> None:
        self.autonomy_mode = autonomy_mode
        self.tracker = ObstacleTracker()
        self.current_mode = "ROVER"
        self.transition_state: str | None = None
        self.clear_landing_streak = 0

    # This method lets ROS parameter updates change the autonomy policy cleanly.
    def set_autonomy_mode(self, autonomy_mode: AutonomyMode) -> None:
        self.autonomy_mode = autonomy_mode

    # This method resets short-term runtime state for a fresh session or mission.
    def reset(self, current_mode: str = "ROVER") -> None:
        self.tracker = ObstacleTracker()
        self.current_mode = current_mode.upper()
        self.transition_state = None
        self.clear_landing_streak = 0

    # This method runs one full perceive-decide-gate-transition cycle.
    def step(self, state: Dict[str, Any]) -> Dict[str, Any]:
        tracking_info = self.tracker.update(state)
        terrain_info = get_terrain_warning(state)
        controls = build_effective_controls(self.autonomy_mode, state.get("controls", {}))

        self.clear_landing_streak = update_clear_landing_streak(
            current_mode=self.current_mode,
            tracking_info=tracking_info,
            terrain_info=terrain_info,
            existing_streak=self.clear_landing_streak,
        )

        result = decide_action(
            current_mode=self.current_mode,
            tracking_info=tracking_info,
            terrain_info=terrain_info,
            transition_state=self.transition_state,
            clear_landing_streak=self.clear_landing_streak,
            confirmed_for_drone=bool(controls.get("approve_drone_switch", False)),
            confirmed_for_landing=bool(controls.get("approve_landing", False)),
            confirmed_for_rover=bool(controls.get("approve_rover_switch", False)),
        )

        result = apply_autonomy_mode(result, self.autonomy_mode)

        new_mode, new_transition_state = apply_mode_transition(
            current_mode=self.current_mode,
            transition_state=self.transition_state,
            result=result,
        )

        debug = {
            "tracking_info": tracking_info,
            "terrain_info": terrain_info,
            "mode_before": self.current_mode,
            "mode_after": new_mode,
            "transition_state_before": self.transition_state,
            "transition_state_after": new_transition_state,
            "clear_landing_streak": self.clear_landing_streak,
            "effective_controls": controls,
        }

        self.current_mode = new_mode
        self.transition_state = new_transition_state

        if self.current_mode != "DRONE":
            self.clear_landing_streak = 0

        return {"result": result, "debug": debug}
