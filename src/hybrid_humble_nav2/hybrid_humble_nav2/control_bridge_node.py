"""Simple bridge from high-level hybrid actions to rover/drone command topics."""
from __future__ import annotations

import json

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - This keeps offline tools import-safe.
    rclpy = None
    Node = object  # type: ignore[misc,assignment]
    Twist = object  # type: ignore[misc,assignment]
    String = object  # type: ignore[misc,assignment]

from .actions import DRIVE_FORWARD, MOVE_BACKWARD, STOP, TURN_LEFT, TURN_RIGHT
from .config import DEFAULT_ACTION_TOPIC


# This node demonstrates how to consume the high-level JSON decision output.
class ControlBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("control_bridge_node")
        self.declare_parameter("action_topic", DEFAULT_ACTION_TOPIC)
        self.declare_parameter("rover_cmd_topic", "/cmd_vel")
        self.declare_parameter("drone_cmd_topic", "/drone_command")

        action_topic = str(self.get_parameter("action_topic").value)
        rover_cmd_topic = str(self.get_parameter("rover_cmd_topic").value)
        drone_cmd_topic = str(self.get_parameter("drone_cmd_topic").value)

        # This subscription listens for high-level actions from the hybrid engine.
        self.action_sub = self.create_subscription(String, action_topic, self.on_action, 10)

        # This publisher sends rover-friendly Twist commands for ground motion.
        self.rover_pub = self.create_publisher(Twist, rover_cmd_topic, 10)

        # This publisher sends simple string commands for a future drone bridge.
        self.drone_pub = self.create_publisher(String, drone_cmd_topic, 10)

    # This callback maps the shared action vocabulary into example downstream commands.
    def on_action(self, msg: String) -> None:
        payload = json.loads(msg.data)
        action = payload.get("current_action", "")
        mode = payload.get("current_mode", "ROVER")
        if mode == "ROVER":
            twist = Twist()
            if action == DRIVE_FORWARD:
                twist.linear.x = 0.2
            elif action == MOVE_BACKWARD:
                twist.linear.x = -0.15
            elif action == TURN_LEFT:
                twist.angular.z = 0.6
            elif action == TURN_RIGHT:
                twist.angular.z = -0.6
            elif action == STOP:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.rover_pub.publish(twist)
        else:
            drone_msg = String()
            drone_msg.data = action
            self.drone_pub.publish(drone_msg)


# This entry point launches the bridge node in a standard Humble rclpy process.
def main(args: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python packages are not installed in this environment.")
    rclpy.init(args=args)
    node = ControlBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# This guard lets ROS console entry points call the node directly.
if __name__ == "__main__":
    main()
