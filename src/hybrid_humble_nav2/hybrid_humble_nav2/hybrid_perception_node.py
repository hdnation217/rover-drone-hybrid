"""ROS 2 Humble node that subscribes to detections and scan and publishes obstacles and actions."""
from __future__ import annotations

import json
from typing import Any, Dict

from .config import (
    DEFAULT_ACTION_TOPIC,
    DEFAULT_AUTONOMY_MODE,
    DEFAULT_DETECTIONS_TOPIC,
    DEFAULT_OBSTACLES_TOPIC,
    DEFAULT_SCAN_TOPIC,
    AutonomyMode,
)
from .hybrid_engine import HybridDecisionEngine
from .ros_state_adapter import RosStateAdapter

try:
    import rclpy
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray
except ImportError:  # pragma: no cover - This keeps offline tools import-safe.
    rclpy = None
    Node = object  # type: ignore[misc,assignment]
    SetParametersResult = None  # type: ignore[assignment]
    LaserScan = object  # type: ignore[misc,assignment]
    String = object  # type: ignore[misc,assignment]
    Detection2DArray = object  # type: ignore[misc,assignment]


# This node adapts ROS messages into the shared hybrid decision engine.
class HybridPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_perception_node")

        # These parameters keep topic names easy to remap in launch files.
        self.declare_parameter("detections_topic", DEFAULT_DETECTIONS_TOPIC)
        self.declare_parameter("scan_topic", DEFAULT_SCAN_TOPIC)
        self.declare_parameter("obstacles_topic", DEFAULT_OBSTACLES_TOPIC)
        self.declare_parameter("action_topic", DEFAULT_ACTION_TOPIC)
        self.declare_parameter("autonomy_mode", DEFAULT_AUTONOMY_MODE.value)
        self.declare_parameter("front_sector_degrees", 70.0)
        self.declare_parameter("obstacle_distance_m", 0.8)
        self.declare_parameter("route_valid", True)
        self.declare_parameter("goal_reached", False)
        self.declare_parameter("terrain", "clear")

        autonomy_mode = AutonomyMode(self.get_parameter("autonomy_mode").value)
        front_sector = float(self.get_parameter("front_sector_degrees").value)
        obstacle_distance = float(self.get_parameter("obstacle_distance_m").value)

        # The adapter holds the latest sensor state from detections and lidar.
        self.adapter = RosStateAdapter(front_sector_degrees=front_sector, obstacle_distance_m=obstacle_distance)
        self.adapter.set_route_valid(bool(self.get_parameter("route_valid").value))
        self.adapter.set_goal_reached(bool(self.get_parameter("goal_reached").value))
        self.adapter.set_terrain(str(self.get_parameter("terrain").value))

        # The engine owns mode transitions and autonomy gating across callbacks.
        self.engine = HybridDecisionEngine(autonomy_mode)

        detections_topic = str(self.get_parameter("detections_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        obstacles_topic = str(self.get_parameter("obstacles_topic").value)
        action_topic = str(self.get_parameter("action_topic").value)

        # The detections subscription receives vision_msgs from Isaac/YOLO.
        self.detections_sub = self.create_subscription(Detection2DArray, detections_topic, self.on_detections, 10)

        # The lidar subscription receives LaserScan from the LDLIDAR driver.
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)

        # This publisher emits a filtered obstacle scan that Nav2 can use as an observation source.
        self.obstacles_pub = self.create_publisher(LaserScan, obstacles_topic, 10)

        # This publisher emits the engine decision as JSON for downstream bridges.
        self.action_pub = self.create_publisher(String, action_topic, 10)

        # This publisher exposes rich debug state for quick inspection and logging.
        self.debug_pub = self.create_publisher(String, "/hybrid_debug_state", 10)

        # This callback keeps autonomy mode and mission flags adjustable at runtime.
        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.get_logger().info(
            f"HybridPerceptionNode started with detections='{detections_topic}', scan='{scan_topic}', "
            f"obstacles='{obstacles_topic}', action='{action_topic}', autonomy='{autonomy_mode.value}'."
        )

    # This callback updates the latest detections but waits for scan to publish a fused step.
    def on_detections(self, msg: Any) -> None:
        self.adapter.update_detections(msg)

    # This callback fuses the latest scan and detections, then publishes obstacles and actions.
    def on_scan(self, msg: Any) -> None:
        self.adapter.update_scan(msg)
        obstacle_scan = self.adapter.build_obstacle_scan(msg)
        if obstacle_scan is not None:
            self.obstacles_pub.publish(obstacle_scan)

        state = self.adapter.build_state(controls={})
        step_output = self.engine.step(state)

        action_msg = String()
        action_msg.data = json.dumps(step_output["result"])
        self.action_pub.publish(action_msg)

        debug_msg = String()
        debug_msg.data = json.dumps({
            "state": state,
            "debug": step_output["debug"],
            "result": step_output["result"],
        })
        self.debug_pub.publish(debug_msg)

    # This callback lets ROS parameters modify autonomy and mission flags live.
    def on_parameter_update(self, params: list[Any]) -> Any:
        for param in params:
            if param.name == "autonomy_mode":
                self.engine.set_autonomy_mode(AutonomyMode(str(param.value)))
            elif param.name == "route_valid":
                self.adapter.set_route_valid(bool(param.value))
            elif param.name == "goal_reached":
                self.adapter.set_goal_reached(bool(param.value))
            elif param.name == "terrain":
                self.adapter.set_terrain(str(param.value))
        return SetParametersResult(successful=True)


# This entry point launches the ROS node in a standard Humble rclpy process.
def main(args: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are not installed in this environment.")
    rclpy.init(args=args)
    node = HybridPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# This guard lets ROS console entry points call the node directly.
if __name__ == "__main__":
    main()
