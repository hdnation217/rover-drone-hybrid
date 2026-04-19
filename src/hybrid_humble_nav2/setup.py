from setuptools import setup

package_name = "hybrid_humble_nav2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "sample_input.json", "README_SUMMARY.md"]),
        (f"share/{package_name}/launch", ["launch/hybrid_perception.launch.py"]),
        (f"share/{package_name}/config", ["config/nav2_obstacle_layer_example.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Darsh",
    maintainer_email="you@example.com",
    description="Hybrid rover/drone decision package for ROS 2 Humble, LDLIDAR, YOLO detections, and Nav2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hybrid_perception_node = hybrid_humble_nav2.hybrid_perception_node:main",
            "control_bridge_node = hybrid_humble_nav2.control_bridge_node:main",
        ],
    },
)
