"""Launch Gazebo, control, MoveIt, RGB-D perception, RViz, and the task."""

from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = Path(
        get_package_share_directory("pick_place_description")
    )
    perception_share = Path(
        get_package_share_directory("pick_place_perception")
    )
    tasks_share = Path(get_package_share_directory("pick_place_tasks"))
    workspace_root = Path(get_package_prefix("pick_place_tasks")).parents[1]
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))

    robot_xacro = description_share / "urdf" / "pick_place.urdf.xacro"
    world_file = description_share / "worlds" / "pick_place_workcell.sdf"
    rviz_file = description_share / "rviz" / "pick_place.rviz"

    moveit_config = (
        MoveItConfigsBuilder(
            "panda",
            package_name="pick_place_moveit_config",
        )
        .robot_description(file_path=str(robot_xacro))
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )
    moveit_parameters = moveit_config.to_dict()

    use_rviz = LaunchConfiguration("rviz")
    auto_start = LaunchConfiguration("auto_start")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [LaunchConfiguration("gz_flags"), f" {world_file}"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "panda_workcell",
            "-topic",
            "robot_description",
            "-allow_renaming",
            "false",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pick_place_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/gripper/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/gripper/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/gripper/attached@std_msgs/msg/String[gz.msgs.StringMsg",
            "/world/pick_place_workcell/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "panda_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )
    hand_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "hand_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_parameters, {"use_sim_time": True}],
    )

    perception = Node(
        package="pick_place_perception",
        executable="object_localizer",
        name="object_localizer",
        output="screen",
        parameters=[
            str(perception_share / "config" / "object_localizer.yaml"),
            {"use_sim_time": True},
        ],
    )

    task = Node(
        package="pick_place_tasks",
        executable="vision_pick_place_task",
        name="vision_pick_place_task",
        output="screen",
        parameters=[
            moveit_parameters,
            str(tasks_share / "config" / "task.yaml"),
            {
                "use_sim_time": True,
                "auto_start": ParameterValue(auto_start, value_type=bool),
                "log_file": LaunchConfiguration("log_file"),
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="pick_place_rviz",
        output="screen",
        arguments=["-d", str(rviz_file)],
        parameters=[moveit_parameters, {"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    # Timers encode startup dependencies: Gazebo -> robot plugin/controller
    # manager -> controllers/MoveIt -> perception/task UI.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_flags", default_value="-r -v 3",
                description="Gazebo flags; use '-s -r -v 3' for server-only tests",
            ),
            DeclareLaunchArgument(
                "log_file",
                default_value=str(workspace_root / "logs" / "pick_place_runs.csv"),
                description="CSV run log destination",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz 2"
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Automatically run one pick-and-place cycle",
            ),
            gazebo,
            robot_state_publisher,
            bridge,
            TimerAction(period=2.0, actions=[spawn_robot]),
            TimerAction(
                period=5.0,
                actions=[joint_state_spawner, arm_spawner, hand_spawner],
            ),
            TimerAction(period=7.0, actions=[move_group, perception]),
            TimerAction(period=10.0, actions=[rviz]),
            TimerAction(period=12.0, actions=[task]),
        ]
    )
