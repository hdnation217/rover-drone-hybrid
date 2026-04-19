"""Offline simulator entry point."""
from pathlib import Path
from typing import Any, Dict, List

from .config import DEFAULT_AUTONOMY_MODE
from .hybrid_engine import HybridDecisionEngine
from .parser import load_input_sequence, load_metadata


# This helper formats one simulator step into readable console output.
def format_result_line(
    state: Dict[str, Any],
    tracking_info: Dict[str, Any],
    terrain_info: Dict[str, Any],
    result: Dict[str, Any],
    clear_landing_streak: int,
    autonomy_mode: str,
) -> List[str]:
    obstacle = tracking_info.get("main_obstacle")
    obstacle_label = obstacle["label"] if obstacle else "none"
    obstacle_position = tracking_info.get("obstacle_position") or "none"

    return [
        f"Frame: {state['frame_id']}",
        f"Terrain: {terrain_info['terrain_type']}",
        f"Obstacle: {obstacle_label}",
        f"Position: {obstacle_position}",
        f"Blocking path: {tracking_info.get('blocking_forward_path', False)}",
        f"Route valid: {tracking_info.get('route_valid', True)}",
        f"Goal reached: {tracking_info.get('goal_reached', False)}",
        f"Moving: {tracking_info.get('is_moving', False)}",
        f"Mode: {result['current_mode']}",
        f"Action: {result['current_action']}",
        f"Requested mode: {result['requested_mode']}",
        f"Transition state: {result['transition_state']}",
        f"Confirmation required: {result['confirmation_required']}",
        f"Confirmed: {result['confirmed']}",
        f"Landing clear streak: {clear_landing_streak}",
        f"Reason: {result['reason']}",
        f"Autonomy mode: {autonomy_mode}",
        f"Execution allowed: {result['execution_allowed']}",
        f"Human approval required: {result['human_approval_required']}",
        f"Execution reason: {result['execution_reason']}",
    ]


# This helper runs the offline frame sequence through the shared engine.
def run_sequence(input_path: Path) -> List[Dict[str, Any]]:
    frames = load_input_sequence(str(input_path))
    metadata = load_metadata(str(input_path))

    engine = HybridDecisionEngine(DEFAULT_AUTONOMY_MODE)
    engine.reset(current_mode=metadata["initial_mode"])
    history: List[Dict[str, Any]] = []

    for state in frames:
        step_output = engine.step(state)
        history.append({
            "state": state,
            "result": step_output["result"],
            "debug": step_output["debug"],
        })

    return history


# This entry point prints an offline simulator trace for debugging.
def main() -> None:
    project_dir = Path(__file__).resolve().parent.parent
    input_path = project_dir / "sample_input.json"

    history = run_sequence(input_path)

    print("=== Rover-to-Drone Hybrid Decision System ===")
    for item in history:
        lines = format_result_line(
            state=item["state"],
            tracking_info=item["debug"]["tracking_info"],
            terrain_info=item["debug"]["terrain_info"],
            result=item["result"],
            clear_landing_streak=item["debug"]["clear_landing_streak"],
            autonomy_mode=DEFAULT_AUTONOMY_MODE.value,
        )
        print("\n---")
        for line in lines:
            print(line)
        print(f"Mode after frame: {item['debug']['mode_after']}")
        print(f"Transition after frame: {item['debug']['transition_state_after']}")
    print("\n=== End of Run ===")


# This guard lets the simulator run directly as a script for quick checks.
if __name__ == "__main__":
    main()
