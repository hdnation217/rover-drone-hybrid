"""Offline JSON parser for regression tests and demos."""
import json
from typing import Any, Dict, List


# These keys let the parser accept a few common bounding-box names.
BOX_KEYS = ("box", "bbox", "xyxy")

# These keys let the parser accept a few common label names.
LABEL_KEYS = ("label", "class_name", "name")

# These keys let the parser accept a few common confidence names.
CONFIDENCE_KEYS = ("confidence", "score", "conf")

# These keys let the parser accept a few common detection container names.
DETECTIONS_KEYS = ("detections", "objects")


# This helper loads and validates the top-level JSON structure.
def load_raw_json(file_path: str) -> Dict[str, Any]:
    with open(file_path, "r", encoding="utf-8") as file:
        data = json.load(file)

    if not isinstance(data, dict):
        raise ValueError("Top-level JSON must be an object.")

    return data


# This helper pulls out a normalized [x1, y1, x2, y2] box.
def _extract_box(detection: Dict[str, Any], index: int) -> List[int]:
    for key in BOX_KEYS:
        if key in detection:
            box = detection[key]
            break
    else:
        raise ValueError(
            f"Detection at index {index} is missing a box field. "
            f"Expected one of: {', '.join(BOX_KEYS)}."
        )

    if not isinstance(box, list) or len(box) != 4:
        raise ValueError(
            f"Detection at index {index} must have box data as [x1, y1, x2, y2]."
        )

    if not all(isinstance(value, (int, float)) for value in box):
        raise ValueError(
            f"Detection at index {index} has non-numeric values in its box field."
        )

    x1, y1, x2, y2 = [int(value) for value in box]
    if x2 <= x1 or y2 <= y1:
        raise ValueError(
            f"Detection at index {index} has an invalid box: x2 must be > x1 and y2 must be > y1."
        )

    return [x1, y1, x2, y2]


# This helper picks the first matching key from a set of aliases.
def _extract_first_present(source: Dict[str, Any], keys: tuple[str, ...]) -> Any:
    for key in keys:
        if key in source:
            return source[key]
    return None


# This helper normalizes one detection object into internal format.
def _validate_detection(detection: Dict[str, Any], index: int) -> Dict[str, Any]:
    if not isinstance(detection, dict):
        raise ValueError(f"Detection at index {index} must be a dictionary.")

    label = _extract_first_present(detection, LABEL_KEYS)
    if not isinstance(label, str) or not label.strip():
        raise ValueError(
            f"Detection at index {index} has an invalid label. "
            f"Expected one of: {', '.join(LABEL_KEYS)}."
        )
    label = label.strip().lower()

    confidence = _extract_first_present(detection, CONFIDENCE_KEYS)
    if not isinstance(confidence, (int, float)):
        raise ValueError(
            f"Detection at index {index} has an invalid confidence. "
            f"Expected one of: {', '.join(CONFIDENCE_KEYS)}."
        )
    confidence = float(confidence)
    if not (0.0 <= confidence <= 1.0):
        raise ValueError(
            f"Detection at index {index} has confidence outside [0, 1]."
        )

    box = _extract_box(detection, index)

    return {
        "label": label,
        "confidence": confidence,
        "box": box,
    }


# This helper extracts image dimensions for geometry-based fallback logic.
def _extract_image_dimensions(raw_frame: Dict[str, Any]) -> tuple[int, int]:
    image_size = raw_frame.get("image_size")
    width = None
    height = None

    if isinstance(image_size, dict):
        width = image_size.get("width")
        height = image_size.get("height")

    if width is None:
        width = raw_frame.get("width", raw_frame.get("image_width"))
    if height is None:
        height = raw_frame.get("height", raw_frame.get("image_height"))

    if not isinstance(width, int) or width <= 0:
        raise ValueError(
            "Image width must be a positive integer. Supported keys: "
            "image_size.width, width, image_width."
        )
    if not isinstance(height, int) or height <= 0:
        raise ValueError(
            "Image height must be a positive integer. Supported keys: "
            "image_size.height, height, image_height."
        )

    return width, height


# This helper normalizes one frame into the shared engine state format.
def _normalize_frame(raw_frame: Dict[str, Any], default_frame_id: int = 0) -> Dict[str, Any]:
    if not isinstance(raw_frame, dict):
        raise ValueError("Each frame must be an object.")

    frame_id = raw_frame.get("frame_id", default_frame_id)
    if not isinstance(frame_id, int):
        raise ValueError("'frame_id' must be an integer.")

    width, height = _extract_image_dimensions(raw_frame)

    terrain = raw_frame.get("terrain", "clear")
    if not isinstance(terrain, str):
        raise ValueError("'terrain' must be a string.")
    terrain = terrain.strip().lower()

    detections = _extract_first_present(raw_frame, DETECTIONS_KEYS)
    if detections is None:
        detections = []
    if not isinstance(detections, list):
        raise ValueError(
            f"Detections must be a list. Supported keys: {', '.join(DETECTIONS_KEYS)}."
        )

    cleaned_detections: List[Dict[str, Any]] = []
    for index, detection in enumerate(detections):
        cleaned_detections.append(_validate_detection(detection, index))

    controls = raw_frame.get("controls", {})
    if controls is None:
        controls = {}
    if not isinstance(controls, dict):
        raise ValueError("'controls' must be an object when provided.")

    # Extra planner-aware fields let offline tests mimic ROS+Nav2 behavior.
    path_blocked = raw_frame.get("path_blocked")
    route_valid = raw_frame.get("route_valid", True)
    goal_reached = raw_frame.get("goal_reached", False)

    return {
        "frame_id": frame_id,
        "image_width": width,
        "image_height": height,
        "terrain": terrain,
        "detections": cleaned_detections,
        "controls": controls,
        "path_blocked": path_blocked,
        "route_valid": bool(route_valid),
        "goal_reached": bool(goal_reached),
    }


# This helper loads one or more frames for offline simulation.
def load_input_sequence(file_path: str) -> List[Dict[str, Any]]:
    raw_data = load_raw_json(file_path)

    if "frames" in raw_data:
        frames = raw_data["frames"]
        if not isinstance(frames, list) or not frames:
            raise ValueError("'frames' must be a non-empty list.")
        return [_normalize_frame(frame, default_frame_id=i + 1) for i, frame in enumerate(frames)]

    return [_normalize_frame(raw_data)]


# This helper loads initial simulator metadata like starting vehicle mode.
def load_metadata(file_path: str) -> Dict[str, Any]:
    raw_data = load_raw_json(file_path)
    initial_mode = raw_data.get("initial_mode", "ROVER")
    if not isinstance(initial_mode, str):
        raise ValueError("'initial_mode' must be a string.")
    initial_mode = initial_mode.strip().upper()
    if initial_mode not in {"ROVER", "DRONE"}:
        raise ValueError("'initial_mode' must be either 'ROVER' or 'DRONE'.")
    return {"initial_mode": initial_mode}
