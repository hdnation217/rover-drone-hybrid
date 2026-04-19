"""Obstacle selection and motion tracking."""
from typing import Any, Dict, List, Optional, Tuple


# This label list defines which detections should be treated as obstacles.
OBSTACLE_LABELS = {
    "rock",
    "tree",
    "wall",
    "person",
    "barrier",
    "pole",
    "chair",
    "bench",
    "car",
    "bicycle",
    "motorcycle",
    "backpack",
    "box",
    "truck",
}

# This threshold rejects weak detections before they affect control.
MIN_CONFIDENCE = 0.60

# This threshold rejects tiny boxes that are usually noise.
MIN_AREA_RATIO = 0.02

# This threshold separates moving from mostly static obstacles.
DEFAULT_MOVEMENT_THRESHOLD = 20.0

# This threshold keeps the history tied to the same object instance.
SAME_OBJECT_DISTANCE_THRESHOLD = 60.0


# This tracker maintains short-term obstacle state across frames.
class ObstacleTracker:
    def __init__(
        self,
        max_history: int = 5,
        movement_threshold: float = DEFAULT_MOVEMENT_THRESHOLD,
    ) -> None:
        self.max_history = max_history
        self.movement_threshold = movement_threshold
        self.center_history: List[Tuple[float, float]] = []
        self.last_label: Optional[str] = None

    # This helper computes pixel area for ranking and filtering boxes.
    def _box_area(self, box: List[int]) -> int:
        x1, y1, x2, y2 = box
        return max(0, x2 - x1) * max(0, y2 - y1)

    # This helper computes the box center for motion tracking.
    def _get_box_center(self, box: List[int]) -> Tuple[float, float]:
        x1, y1, x2, y2 = box
        return (x1 + x2) / 2, (y1 + y2) / 2

    # This helper computes Euclidean distance between tracked centers.
    def _distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return (dx ** 2 + dy ** 2) ** 0.5

    # This helper estimates obstacle side from image geometry.
    def _get_box_position(self, box: List[int], image_width: int) -> str:
        x1, _, x2, _ = box
        center_x = (x1 + x2) / 2
        if center_x < image_width / 3:
            return "left"
        if center_x < 2 * image_width / 3:
            return "center"
        return "right"

    # This helper is the fallback when no planner-aware blockage flag exists.
    def _overlaps_forward_path(self, box: List[int], image_width: int) -> bool:
        x1, _, x2, _ = box
        path_left = int(image_width * 0.35)
        path_right = int(image_width * 0.65)
        return x2 >= path_left and x1 <= path_right

    # This helper decides whether a single detection is worth using.
    def _is_valid_obstacle(
        self,
        detection: Dict[str, Any],
        image_width: int,
        image_height: int,
    ) -> bool:
        label = str(detection.get("label", "")).lower()
        confidence = float(detection.get("confidence", 0.0))
        box = detection.get("box", [0, 0, 0, 0])

        if label not in OBSTACLE_LABELS:
            return False
        if confidence < MIN_CONFIDENCE:
            return False

        frame_area = image_width * image_height
        if frame_area <= 0:
            return False

        area_ratio = self._box_area(box) / frame_area
        return area_ratio >= MIN_AREA_RATIO

    # This helper picks the most important obstacle in the frame.
    def get_main_obstacle(self, state: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        detections = state.get("detections", [])
        image_width = state.get("image_width", 640)
        image_height = state.get("image_height", 480)

        valid_obstacles = [
            detection
            for detection in detections
            if self._is_valid_obstacle(detection, image_width, image_height)
        ]
        if not valid_obstacles:
            return None

        # Planner-aware blockage takes priority over image-center heuristics.
        planner_blocked = state.get("path_blocked")
        if isinstance(planner_blocked, bool):
            if planner_blocked:
                return max(valid_obstacles, key=lambda det: self._box_area(det["box"]))
            return max(valid_obstacles, key=lambda det: self._box_area(det["box"]))

        blocking_obstacles = [
            detection
            for detection in valid_obstacles
            if self._overlaps_forward_path(detection["box"], image_width)
        ]

        pool = blocking_obstacles if blocking_obstacles else valid_obstacles
        return max(pool, key=lambda det: self._box_area(det["box"]))

    # This helper turns a center history into an average movement estimate.
    def _compute_average_movement(self) -> float:
        if len(self.center_history) < 2:
            return 0.0

        total_movement = 0.0
        for i in range(1, len(self.center_history)):
            total_movement += self._distance(
                self.center_history[i - 1],
                self.center_history[i],
            )
        return total_movement / (len(self.center_history) - 1)

    # This helper updates history only when the same object is likely being tracked.
    def _update_history(self, label: str, center: Tuple[float, float]) -> None:
        if not self.center_history:
            self.center_history = [center]
            self.last_label = label
            return

        previous_center = self.center_history[-1]
        same_label = self.last_label == label
        close_enough = self._distance(previous_center, center) <= SAME_OBJECT_DISTANCE_THRESHOLD

        if same_label and close_enough:
            self.center_history.append(center)
            if len(self.center_history) > self.max_history:
                self.center_history.pop(0)
        else:
            self.center_history = [center]

        self.last_label = label

    # This method returns the normalized obstacle state used by the decision engine.
    def update(self, state: Dict[str, Any]) -> Dict[str, Any]:
        main_obstacle = self.get_main_obstacle(state)

        if main_obstacle is None:
            self.center_history.clear()
            self.last_label = None
            return {
                "main_obstacle": None,
                "has_obstacle": False,
                "is_moving": False,
                "is_static": False,
                "current_center": None,
                "average_movement": 0.0,
                "history_length": 0,
                "obstacle_position": None,
                "blocking_forward_path": bool(state.get("path_blocked", False)),
                "scan_blocked": bool(state.get("scan_blocked", False)),
                "route_valid": bool(state.get("route_valid", True)),
                "goal_reached": bool(state.get("goal_reached", False)),
                "obstacle_area_ratio": 0.0,
            }

        image_width = state.get("image_width", 640)
        image_height = state.get("image_height", 480)
        box = main_obstacle["box"]
        label = str(main_obstacle.get("label", "unknown")).lower()
        current_center = self._get_box_center(box)
        obstacle_position = self._get_box_position(box, image_width)
        frame_area = max(1, image_width * image_height)
        obstacle_area_ratio = self._box_area(box) / frame_area

        # Planner-aware blockage overrides the camera-only center-band estimate.
        if isinstance(state.get("path_blocked"), bool):
            blocking_forward_path = bool(state["path_blocked"])
        else:
            blocking_forward_path = self._overlaps_forward_path(box, image_width)

        self._update_history(label, current_center)

        avg_movement = self._compute_average_movement()
        is_moving = avg_movement > self.movement_threshold
        is_static = len(self.center_history) >= 2 and not is_moving

        return {
            "main_obstacle": main_obstacle,
            "has_obstacle": True,
            "is_moving": is_moving,
            "is_static": is_static,
            "current_center": current_center,
            "average_movement": avg_movement,
            "history_length": len(self.center_history),
            "obstacle_position": obstacle_position,
            "blocking_forward_path": blocking_forward_path,
            "scan_blocked": bool(state.get("scan_blocked", False)),
            "route_valid": bool(state.get("route_valid", True)),
            "goal_reached": bool(state.get("goal_reached", False)),
            "obstacle_area_ratio": obstacle_area_ratio,
        }
