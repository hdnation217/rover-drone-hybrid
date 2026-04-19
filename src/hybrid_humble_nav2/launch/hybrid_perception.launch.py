"""Launch file for the hybrid perception node on ROS 2 Humble."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# This launch file makes topic remaps and thresholds easy to tweak from the CLI.
def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("detections_topic", default_value="/detections"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("obstacles_topic", default_value="/obstacles"),
        DeclareLaunchArgument("action_topic", default_value="/hybrid_action"),
        DeclareLaunchArgument("autonomy_mode", default_value="SEMI_AUTONOMOUS"),
        DeclareLaunchArgument("front_sector_degrees", default_value="70.0"),
        DeclareLaunchArgument("obstacle_distance_m", default_value="0.8"),
        Node(
            package="hybrid_humble_nav2",
            executable="hybrid_perception_node",
            name="hybrid_perception_node",
            output="screen",
            parameters=[{
                "detections_topic": LaunchConfiguration("detections_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "obstacles_topic": LaunchConfiguration("obstacles_topic"),
                "action_topic": LaunchConfiguration("action_topic"),
                "autonomy_mode": LaunchConfiguration("autonomy_mode"),
                "front_sector_degrees": LaunchConfiguration("front_sector_degrees"),
                "obstacle_distance_m": LaunchConfiguration("obstacle_distance_m"),
            }],
        ),
    ])
