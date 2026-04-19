"""ROS message adapters for Detection2DArray and LaserScan."""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple


# This label map normalizes detector labels into the obstacle vocabulary used by the engine.
LABEL_MAP = {
    "traffic cone": "barrier",
    "cone": "barrier",
    "person": "person",
    "truck": "truck",
    "car": "car",
    "motorcycle": "motorcycle",
    "bicycle": "bicycle",
    "chair": "chair",
    "bench": "bench",
    "backpack": "backpack",
    "potted plant": "tree",
    "suitcase": "box",
}


# This adapter owns short-lived ROS sensor state before handing it to the engine.
class RosStateAdapter:
    def __init__(self, front_sector_degrees: float = 70.0, obstacle_distance_m: float = 0.8) -> None:
        self.front_sector_degrees = front_sector_degrees
        self.obstacle_distance_m = obstacle_distance_m
        self.latest_scan: Any | None = None
        self.latest_detections: Any | None = None
        self.frame_id = 0
        self.route_valid = True
        self.goal_reached = False
        self.terrain = "clear"

    # This setter stores the latest detection message from YOLO/vision_msgs.
    def update_detections(self, detections_msg: Any) -> None:
        self.latest_detections = detections_msg

    # This setter stores the latest scan message from the lidar driver.
    def update_scan(self, scan_msg: Any) -> None:
        self.latest_scan = scan_msg

    # This setter lets Nav2 feedback update route validity without changing core logic.
    def set_route_valid(self, route_valid: bool) -> None:
        self.route_valid = bool(route_valid)

    # This setter lets higher-level mission logic mark a goal as reached.
    def set_goal_reached(self, goal_reached: bool) -> None:
        self.goal_reached = bool(goal_reached)

    # This setter lets external code provide a terrain label when available.
    def set_terrain(self, terrain: str) -> None:
        self.terrain = str(terrain).strip().lower() or "clear"

    # This helper extracts the best hypothesis from a vision message result array.
    def _extract_label_and_score(self, result: Any) -> Tuple[str, float]:
        hypotheses = getattr(result, "hypothesis", None)
        if hypotheses is not None:
            label = getattr(hypotheses, "class_id", "unknown")
            score = float(getattr(hypotheses, "score", 0.0))
            return str(label), score

        results = getattr(result, "results", None)
        if results:
            hypothesis = results[0].hypothesis
            label = getattr(hypothesis, "class_id", "unknown")
            score = float(getattr(hypothesis, "score", 0.0))
            return str(label), score

        return "unknown", 0.0

    # This helper converts Detection2DArray into the internal detection list.
    def _normalize_detections(self, detections_msg: Any) -> List[Dict[str, Any]]:
        detections: List[Dict[str, Any]] = []
        if detections_msg is None:
            return detections

        for det in getattr(detections_msg, "detections", []):
            bbox = getattr(det, "bbox", None)
            if bbox is None:
                continue
            center = getattr(bbox, "center", None)
            size_x = float(getattr(bbox, "size_x", 0.0))
            size_y = float(getattr(bbox, "size_y", 0.0))
            center_x = float(getattr(getattr(center, "position", center), "x", 0.0))
            center_y = float(getattr(getattr(center, "position", center), "y", 0.0))
            x1 = int(center_x - size_x / 2.0)
            y1 = int(center_y - size_y / 2.0)
            x2 = int(center_x + size_x / 2.0)
            y2 = int(center_y + size_y / 2.0)
            label, score = self._extract_label_and_score(det)
            label = LABEL_MAP.get(label.strip().lower(), label.strip().lower())
            detections.append({
                "label": label,
                "confidence": float(score),
                "box": [x1, y1, x2, y2],
            })
        return detections

    # This helper identifies front-sector obstacles from LaserScan.
    def _scan_blocking_info(self, scan_msg: Any) -> Tuple[bool, float | None, List[int]]:
        if scan_msg is None:
            return False, None, []

        half_sector = math.radians(self.front_sector_degrees / 2.0)
        ranges = list(getattr(scan_msg, "ranges", []))
        angle_min = float(getattr(scan_msg, "angle_min", 0.0))
        angle_increment = float(getattr(scan_msg, "angle_increment", 0.0))

        blocked_indices: List[int] = []
        nearest_distance: float | None = None
        for index, distance in enumerate(ranges):
            angle = angle_min + index * angle_increment
            if abs(angle) > half_sector:
                continue
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance <= self.obstacle_distance_m:
                blocked_indices.append(index)
                nearest_distance = distance if nearest_distance is None else min(nearest_distance, distance)

        return bool(blocked_indices), nearest_distance, blocked_indices

    # This helper builds a filtered LaserScan that Nav2 can consume as obstacle observations.
    def build_obstacle_scan(self, scan_msg: Any) -> Any | None:
        if scan_msg is None:
            return None
        blocked, _, blocked_indices = self._scan_blocking_info(scan_msg)
        if not blocked:
            return scan_msg

        filtered_scan = type(scan_msg)()
        filtered_scan.header = scan_msg.header
        filtered_scan.angle_min = scan_msg.angle_min
        filtered_scan.angle_max = scan_msg.angle_max
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        filtered_scan.range_min = scan_msg.range_min
        filtered_scan.range_max = scan_msg.range_max
        filtered_ranges = [float("inf")] * len(scan_msg.ranges)
        filtered_intensities = [0.0] * len(getattr(scan_msg, "intensities", []))
        for index in blocked_indices:
            filtered_ranges[index] = scan_msg.ranges[index]
            if index < len(filtered_intensities):
                filtered_intensities[index] = scan_msg.intensities[index]
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = filtered_intensities
        return filtered_scan

    # This method emits the shared engine state assembled from the latest ROS messages.
    def build_state(self, controls: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        self.frame_id += 1
        detections = self._normalize_detections(self.latest_detections)
        scan_blocked, nearest_distance, _ = self._scan_blocking_info(self.latest_scan)

        image_width = 640
        image_height = 480
        if self.latest_detections is not None:
            header = getattr(self.latest_detections, "header", None)
            _ = header  # This preserves a spot for future camera metadata use.

        return {
            "frame_id": self.frame_id,
            "image_width": image_width,
            "image_height": image_height,
            "terrain": self.terrain,
            "detections": detections,
            "controls": controls or {},
            "path_blocked": scan_blocked,
            "scan_blocked": scan_blocked,
            "nearest_scan_obstacle_m": nearest_distance,
            "route_valid": self.route_valid,
            "goal_reached": self.goal_reached,
        }
