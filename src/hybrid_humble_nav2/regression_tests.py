"""Basic regression tests for the shared engine."""
from pathlib import Path

from hybrid_humble_nav2.config import AutonomyMode
from hybrid_humble_nav2.hybrid_engine import HybridDecisionEngine
from hybrid_humble_nav2.parser import load_input_sequence


# This helper runs a list of states through the engine and collects only actions.
def _run_actions(states, autonomy_mode=AutonomyMode.SEMI_AUTONOMOUS):
    engine = HybridDecisionEngine(autonomy_mode)
    return [engine.step(state)["result"]["current_action"] for state in states]


# This test checks the rover-stop-backup-escalate sequence from the sample input.
def test_sample_sequence_has_expected_escalation() -> None:
    sample_path = Path(__file__).resolve().parent / "sample_input.json"
    states = load_input_sequence(str(sample_path))
    actions = _run_actions(states)
    assert actions[0] == "STOP"
    assert actions[1] == "STOP"
    assert actions[3] == "SWITCH_TO_DRONE"


# This test checks that a planner-invalid route escalates toward drone mode.
def test_invalid_route_requests_drone() -> None:
    engine = HybridDecisionEngine(AutonomyMode.SEMI_AUTONOMOUS)
    state = {
        "frame_id": 1,
        "image_width": 640,
        "image_height": 480,
        "terrain": "clear",
        "detections": [],
        "controls": {},
        "path_blocked": False,
        "route_valid": False,
        "goal_reached": False,
    }
    result = engine.step(state)["result"]
    assert result["requested_mode"] == "DRONE"


# This test checks that manual mode blocks autonomous execution.
def test_manual_mode_blocks_execution() -> None:
    engine = HybridDecisionEngine(AutonomyMode.MANUAL)
    state = {
        "frame_id": 1,
        "image_width": 640,
        "image_height": 480,
        "terrain": "clear",
        "detections": [],
        "controls": {},
        "path_blocked": False,
        "route_valid": True,
        "goal_reached": False,
    }
    result = engine.step(state)["result"]
    assert result["execution_allowed"] is False


# This test checks that fully autonomous mode self-approves the drone switch.
def test_full_auto_executes_rover_to_drone_transition() -> None:
    engine = HybridDecisionEngine(AutonomyMode.FULLY_AUTONOMOUS)
    blocking_detection = {
        "label": "rock",
        "confidence": 0.95,
        "box": [220, 120, 440, 360],
    }
    state = {
        "frame_id": 1,
        "image_width": 640,
        "image_height": 480,
        "terrain": "clear",
        "detections": [blocking_detection],
        "controls": {},
        "path_blocked": True,
        "route_valid": True,
        "goal_reached": False,
    }

    first = engine.step(state)["result"]
    second = engine.step({**state, "frame_id": 2})["result"]
    third = engine.step({**state, "frame_id": 3})["result"]
    assert first["current_action"] == "STOP"
    assert second["requested_mode"] == "DRONE"
    assert third["current_action"] == "SWITCH_TO_DRONE"
    assert engine.current_mode == "DRONE"


# This test checks that fully autonomous mode can land and switch back to rover.
def test_full_auto_lands_and_returns_to_rover() -> None:
    engine = HybridDecisionEngine(AutonomyMode.FULLY_AUTONOMOUS)
    engine.current_mode = "DRONE"

    clear_state = {
        "frame_id": 1,
        "image_width": 640,
        "image_height": 480,
        "terrain": "flat",
        "detections": [],
        "controls": {},
        "path_blocked": False,
        "route_valid": True,
        "goal_reached": True,
    }

    first = engine.step(clear_state)["result"]
    second = engine.step({**clear_state, "frame_id": 2})["result"]
    third = engine.step({**clear_state, "frame_id": 3})["result"]
    fourth = engine.step({**clear_state, "frame_id": 4})["result"]

    assert first["current_action"] == "HOVER"
    assert second["current_action"] == "LAND"
    assert third["current_action"] == "SWITCH_TO_ROVER"
    assert engine.current_mode == "ROVER"
    assert fourth["current_action"] == "STOP"


# This helper runs the tests when executed as a simple script.
def main() -> None:
    test_sample_sequence_has_expected_escalation()
    test_invalid_route_requests_drone()
    test_manual_mode_blocks_execution()
    test_full_auto_executes_rover_to_drone_transition()
    test_full_auto_lands_and_returns_to_rover()
    print("All regression tests passed.")


# This guard makes the file easy to run without pytest.
if __name__ == "__main__":
    main()
