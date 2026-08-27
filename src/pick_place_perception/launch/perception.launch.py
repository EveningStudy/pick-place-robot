from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config = (
        Path(get_package_share_directory("pick_place_perception"))
        / "config"
        / "object_localizer.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="pick_place_perception",
                executable="object_localizer",
                name="object_localizer",
                output="screen",
                parameters=[str(config), {"use_sim_time": True}],
            )
        ]
    )
