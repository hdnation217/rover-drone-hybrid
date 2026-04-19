"""Master bringup: LiDAR + Camera/YOLO + Brain + Control Bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    autonomy_mode_arg = DeclareLaunchArgument(
        "autonomy_mode",
        default_value="FULLY_AUTONOMOUS",
        description="MANUAL, SEMI_AUTONOMOUS, or FULLY_AUTONOMOUS",
    )
    obstacle_distance_arg = DeclareLaunchArgument(
        "obstacle_distance_m",
        default_value="0.5",
        description="LiDAR front-sector obstacle threshold in meters",
    )
    front_sector_arg = DeclareLaunchArgument(
        "front_sector_degrees",
        default_value="50.0",
        description="LiDAR front-sector arc width in degrees",
    )
    camera_index_arg = DeclareLaunchArgument(
        "camera_index",
        default_value="0",
        description="Index for /dev/videoN",
    )
    camera_rate_arg = DeclareLaunchArgument(
        "camera_rate_hz",
        default_value="10.0",
        description="Rate at which camera_yolo_node publishes detections",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/workspaces/isaac_ros-dev/yolov8n.engine",
        description="Path to YOLO TensorRT engine",
    )

    # LiDAR driver via the package's own launch file
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ldlidar_ros2"),
                "launch",
                "ld14p.launch.py",
            ])
        ]),
    )

    # Camera + YOLO inference node
    camera_node = Node(
        package="rover_perception",
        executable="camera_yolo_node",
        name="camera_yolo_node",
        output="screen",
        parameters=[{
            "camera_index": LaunchConfiguration("camera_index"),
            "model_path": LaunchConfiguration("model_path"),
            "publish_rate_hz": LaunchConfiguration("camera_rate_hz"),
            "confidence_threshold": 0.4,
            "yolo_imgsz": 640,
            "image_width": 640,
            "image_height": 480,
            "frame_id": "camera_link",
            "detections_topic": "/detections",
        }],
    )

    # Brain: fuses scan + detections into hybrid_action
    perception_node = Node(
        package="hybrid_humble_nav2",
        executable="hybrid_perception_node",
        name="hybrid_perception_node",
        output="screen",
        parameters=[{
            "autonomy_mode": LaunchConfiguration("autonomy_mode"),
            "obstacle_distance_m": LaunchConfiguration("obstacle_distance_m"),
            "front_sector_degrees": LaunchConfiguration("front_sector_degrees"),
        }],
    )

    # Bridge: converts hybrid_action into /cmd_vel twist
    control_bridge_node = Node(
        package="hybrid_humble_nav2",
        executable="control_bridge_node",
        name="control_bridge_node",
        output="screen",
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",
        }],
    )

# slam_toolbox config path
    slam_config_path = os.path.join(
        get_package_share_directory('rover_perception'),
        'config',
        'slam_params.yaml'
    )

    # Static TF: odom -> base_link (fake odometry since we have no wheel encoders yet)
    odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        output="screen",
    )



    # Static TF: map -> odom (we'd normally get this from odometry drift; fake for now)
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # slam_toolbox async SLAM node
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config_path],
    )

    return LaunchDescription([
        autonomy_mode_arg,
        obstacle_distance_arg,
        front_sector_arg,
        camera_index_arg,
        camera_rate_arg,
        model_path_arg,
        lidar_launch,
        map_to_odom,           # NEW
        odom_to_base,
        camera_node,
        perception_node,
        control_bridge_node,
        foxglove_bridge,
        slam_node,             # NEW
    ])
