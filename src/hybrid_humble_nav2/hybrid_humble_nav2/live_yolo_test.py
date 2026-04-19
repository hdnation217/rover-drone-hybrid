"""Local OpenCV+YOLO demo harness that shares the same engine as ROS."""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

import cv2
from pynput import keyboard
from ultralytics import YOLO

from .actions import (
    ASCEND,
    DESCEND,
    DRIVE_FORWARD,
    HOVER,
    LAND,
    LANDED,
    MOVE_BACKWARD,
    MOVE_BACKWARD_AIR,
    MOVE_FORWARD_AIR,
    MOVE_LEFT_AIR,
    MOVE_RIGHT_AIR,
    STOP,
    TURN_LEFT,
    TURN_RIGHT,
)
from .config import AutonomyMode
from .hybrid_engine import HybridDecisionEngine
from .manual_instructions import get_overlay_controls_line, get_startup_instructions


# This map normalizes common YOLO classes into your obstacle labels.
LABEL_MAP = {
    "person": "person",
    "car": "car",
    "truck": "truck",
    "bicycle": "bicycle",
    "motorcycle": "motorcycle",
    "bench": "bench",
    "chair": "chair",
    "backpack": "backpack",
    "potted plant": "tree",
    "traffic cone": "barrier",
    "stop sign": "barrier",
    "suitcase": "box",
}

# These defaults keep the demo lightweight on a laptop webcam.
CAMERA_INDEX = 0
CAMERA_WIDTH = 512
CAMERA_HEIGHT = 384
PROCESS_EVERY_N_FRAMES = 4
YOLO_IMGSZ = 640
YOLO_CONF = 0.45
FONT_SCALE = 0.38
FONT_THICKNESS = 1
LINE_STEP = 16


# Global pressed-key state for true hold behavior.
pressed_keys: set[str] = set()


def on_press(key: Any) -> None:
    try:
        if hasattr(key, "char") and key.char is not None:
            pressed_keys.add(key.char.lower())
    except Exception:
        pass


def on_release(key: Any) -> None:
    try:
        if hasattr(key, "char") and key.char is not None:
            pressed_keys.discard(key.char.lower())
    except Exception:
        pass


# This helper lets a user choose a demo autonomy mode at startup.
def choose_autonomy_mode() -> AutonomyMode:
    while True:
        print("\nChoose autonomy mode:")
        print("1 - MANUAL")
        print("2 - SEMI_AUTONOMOUS")
        print("3 - FULLY_AUTONOMOUS")
        choice = input("Enter 1, 2, or 3: ").strip()

        if choice == "1":
            return AutonomyMode.MANUAL
        if choice == "2":
            return AutonomyMode.SEMI_AUTONOMOUS
        if choice == "3":
            return AutonomyMode.FULLY_AUTONOMOUS
        print("Invalid choice. Please enter 1, 2, or 3.")


# This helper converts a YOLO result into the shared engine state format.
def yolo_results_to_state(
    result: Any,
    frame_id: int,
    terrain: str = "clear",
    controls: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    orig_img = result.orig_img
    image_height, image_width = orig_img.shape[:2]

    detections: List[Dict[str, Any]] = []
    names = result.names
    boxes = result.boxes

    if boxes is not None and len(boxes) > 0:
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy()

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = xyxy[i]
            conf = float(confs[i])
            cls_id = int(classes[i])
            raw_label = str(names[cls_id]).lower().strip()
            mapped_label = LABEL_MAP.get(raw_label, raw_label)
            detections.append(
                {
                    "label": mapped_label,
                    "confidence": conf,
                    "box": [int(x1), int(y1), int(x2), int(y2)],
                }
            )

    return {
        "frame_id": frame_id,
        "image_width": image_width,
        "image_height": image_height,
        "terrain": terrain,
        "detections": detections,
        "controls": controls or {},
        "route_valid": True,
        "goal_reached": False,
    }


# This helper extracts confirmation booleans from keyboard input for the demo.
def handle_mode_switch_command(
    key: int,
    runtime_mode: AutonomyMode,
    current_mode: str,
    transition_state: str | None,
    drone_grounded: bool = False,
) -> Dict[str, bool]:
    controls = {
        "approve_drone_switch": False,
        "approve_landing": False,
        "approve_rover_switch": False,
        "manual_toggle_mode": False,
        "manual_invalid_rover_switch": False,
    }

    if runtime_mode == AutonomyMode.MANUAL:
        if key == ord("r"):
            if current_mode == "ROVER":
                controls["manual_toggle_mode"] = True
            elif current_mode == "DRONE" and drone_grounded:
                controls["manual_toggle_mode"] = True
            else:
                controls["manual_invalid_rover_switch"] = True
        return controls

    if runtime_mode == AutonomyMode.SEMI_AUTONOMOUS:
        if current_mode == "ROVER" and key == ord("d"):
            controls["approve_drone_switch"] = True
        elif current_mode == "DRONE" and transition_state == "TO_ROVER_PENDING" and key == ord("r"):
            controls["approve_rover_switch"] = True
        elif current_mode == "DRONE" and key == ord("l"):
            controls["approve_landing"] = True

    return controls


# This helper builds a manual-mode result payload for the OpenCV demo only.
def build_manual_result(current_mode: str, manual_action: str, status_message: str = "") -> Dict[str, Any]:
    requested_mode = "ROVER" if manual_action == LAND else None
    reason = f"Human manual command: {manual_action}."
    if status_message:
        reason = f"{reason} {status_message}"
    return {
        "current_mode": current_mode,
        "current_action": manual_action,
        "requested_mode": requested_mode,
        "transition_state": None,
        "reason": reason,
        "confirmation_required": False,
        "confirmed": True,
        "execution_allowed": True,
        "human_approval_required": False,
        "execution_reason": "Manual mode is active. Human command is being executed.",
    }


# This helper draws the current state on the webcam frame for debugging.
def draw_overlay(
    frame: Any,
    tracking_info: Dict[str, Any],
    result: Dict[str, Any],
    current_mode: str,
    fps: float,
    terrain: str,
    runtime_mode: AutonomyMode,
    drone_grounded: bool,
    status_message: str,
) -> Any:
    overlay = frame.copy()
    main_obstacle = tracking_info.get("main_obstacle")
    if main_obstacle:
        x1, y1, x2, y2 = main_obstacle["box"]
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{main_obstacle['label']} {main_obstacle['confidence']:.2f}"
        cv2.putText(
            overlay,
            label,
            (x1, max(14, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            FONT_SCALE,
            (0, 255, 0),
            FONT_THICKNESS,
            cv2.LINE_AA,
        )

    controls_line = get_overlay_controls_line(runtime_mode, current_mode)
    lines = [
        f"Mode type: {runtime_mode.value}",
        f"Vehicle mode: {current_mode}",
        f"Action: {result['current_action']}",
        f"Requested: {result.get('requested_mode')}",
        f"Allow: {result.get('execution_allowed')}",
        f"Need approval: {result.get('human_approval_required')}",
        f"Terrain: {terrain}",
        f"FPS: {fps:.1f}",
    ]
    if current_mode == "DRONE":
        lines.append(f"Drone grounded: {drone_grounded}")
    if status_message:
        lines.append(f"Status: {status_message}")
    lines.append(controls_line)

    y = 16
    for line in lines:
        cv2.putText(
            overlay,
            line,
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            FONT_SCALE,
            (255, 255, 255),
            FONT_THICKNESS,
            cv2.LINE_AA,
        )
        y += LINE_STEP
    return overlay


def get_manual_action(current_mode: str, drone_grounded: bool) -> str:
    if current_mode == "ROVER":
        if "w" in pressed_keys:
            return DRIVE_FORWARD
        if "a" in pressed_keys:
            return TURN_LEFT
        if "s" in pressed_keys:
            return MOVE_BACKWARD
        if "d" in pressed_keys:
            return TURN_RIGHT
        return STOP

    if drone_grounded:
        return LANDED

    if "w" in pressed_keys:
        return MOVE_FORWARD_AIR
    if "a" in pressed_keys:
        return MOVE_LEFT_AIR
    if "s" in pressed_keys:
        return MOVE_BACKWARD_AIR
    if "d" in pressed_keys:
        return MOVE_RIGHT_AIR
    if ">" in pressed_keys:
        return ASCEND
    if "<" in pressed_keys:
        return DESCEND
    return HOVER


# This entry point runs the local camera demo while reusing the shared engine.
def main() -> None:
    runtime_mode = choose_autonomy_mode()
    print()
    print(get_startup_instructions(runtime_mode))

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    model = YOLO("/workspaces/isaac_ros-dev/yolov8n.engine", task="detect")
    print("Opening webcam...")
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print("Default camera open failed, trying DirectShow...")
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("DirectShow failed, trying Media Foundation...")
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_MSMF)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)

    time.sleep(1.0)

    if not cap.isOpened():
        listener.stop()
        raise RuntimeError("Could not open webcam.")

    print("Webcam opened successfully.")

    engine = HybridDecisionEngine(runtime_mode)
    frame_id = 0
    terrain = "clear"
    drone_grounded = False
    status_message = ""
    last_yolo_result = None
    fps = 0.0
    prev_time = time.time()

    print(f"Live YOLO test started in {runtime_mode.value}.")
    print("Press q to quit.")

    cv2.namedWindow("Live YOLO Hybrid Test", cv2.WINDOW_NORMAL)
    print("OpenCV window created.")

    try:
        while True:
            ok, frame = cap.read()
            if frame_id == 0:
                print(f"First frame read status: ok={ok}")
            if not ok or frame is None:
                print("Failed to read frame from webcam.")
                break

            frame_id += 1
            if frame_id <= 5:
                preview = frame.copy()
                cv2.putText(
                    preview,
                    f"Camera preview frame {frame_id}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Live YOLO Hybrid Test", preview)
                cv2.waitKey(1)

            now = time.time()
            dt = now - prev_time
            prev_time = now
            if dt > 0:
                instant_fps = 1.0 / dt
                fps = instant_fps if fps == 0.0 else (0.85 * fps + 0.15 * instant_fps)

            if frame_id % PROCESS_EVERY_N_FRAMES == 0 or last_yolo_result is None:
                if frame_id == 1:
                    print("Running first YOLO inference...")
                results = model(frame, imgsz=YOLO_IMGSZ, conf=YOLO_CONF, verbose=False, rect=True)
                last_yolo_result = results[0]
                if frame_id == 1:
                    print("First YOLO inference completed.")

            if last_yolo_result is None:
                continue

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("t"):
                terrain = "flat" if terrain == "clear" else "clear"

            decision_result: Dict[str, Any]
            current_mode = engine.current_mode
            transition_state = engine.transition_state

            if runtime_mode == AutonomyMode.MANUAL:
                status_message = ""
                switch_controls = handle_mode_switch_command(
                    key, runtime_mode, current_mode, transition_state, drone_grounded
                )

                if switch_controls["manual_invalid_rover_switch"]:
                    status_message = "Cannot switch to rover while drone is still in the air. Land first with L."

                if switch_controls["manual_toggle_mode"]:
                    if current_mode == "ROVER":
                        engine.current_mode = "DRONE"
                        drone_grounded = False
                        status_message = "Switched from rover mode to drone mode."
                    else:
                        engine.current_mode = "ROVER"
                        drone_grounded = False
                        status_message = "Switched from drone mode to rover mode after landing."

                current_mode = engine.current_mode

                if current_mode == "DRONE" and "l" in pressed_keys and not drone_grounded:
                    drone_grounded = True
                    manual_action = LAND
                    status_message = "Drone has landed. Press R to switch back to rover mode."
                else:
                    manual_action = get_manual_action(current_mode, drone_grounded)

                decision_result = build_manual_result(
                    engine.current_mode,
                    manual_action,
                    status_message=status_message,
                )
                tracking_info = engine.tracker.update(
                    yolo_results_to_state(last_yolo_result, frame_id, terrain, {})
                )
            else:
                status_message = ""
                switch_controls = handle_mode_switch_command(
                    key,
                    runtime_mode,
                    current_mode,
                    transition_state,
                    drone_grounded=False,
                )
                state = yolo_results_to_state(
                    last_yolo_result,
                    frame_id,
                    terrain,
                    controls={
                        "approve_drone_switch": switch_controls["approve_drone_switch"],
                        "approve_landing": switch_controls["approve_landing"],
                        "approve_rover_switch": switch_controls["approve_rover_switch"],
                    },
                )
                step_output = engine.step(state)
                decision_result = step_output["result"]
                tracking_info = step_output["debug"]["tracking_info"]

            display = draw_overlay(
                frame,
                tracking_info,
                decision_result,
                engine.current_mode,
                fps,
                terrain,
                runtime_mode,
                drone_grounded,
                status_message,
            )
            cv2.imshow("Live YOLO Hybrid Test", display)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        listener.stop()


# This guard lets the demo run directly from a terminal.
if __name__ == "__main__":
    main()
