from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config = (
        Path(get_package_share_directory("pick_place_tasks"))
        / "config"
        / "task.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="pick_place_tasks",
                executable="vision_pick_place_task",
                name="vision_pick_place_task",
                output="screen",
                parameters=[str(config), {"use_sim_time": True}],
            )
        ]
    )
