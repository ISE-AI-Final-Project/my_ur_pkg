import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    start_delay = float(LaunchConfiguration("start_delay").perform(context))
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

    # Compute delayed times
    move_group_delay = start_delay + 4
    rviz_delay = start_delay + 8

    # Paths to launch files
    cell_control_launch = os.path.join(
        get_package_share_directory("my_robot_cell_control"),
        "launch",
        "start_robot.launch.py",
    )

    move_group_launch = os.path.join(
        get_package_share_directory("my_robot_cell_moveit_config"),
        "launch",
        "move_group.launch.py",
    )

    moveit_rviz_launch = os.path.join(
        get_package_share_directory("my_robot_cell_moveit_config"),
        "launch",
        "custom_moveit_rviz.launch.py",
    )

    return [
        # 2. Start UR ROS Driver (after start_delay)
        TimerAction(
            period=start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cell_control_launch),
                    launch_arguments={
                        "ur_type": ur_type,
                        "robot_ip": robot_ip,
                        "launch_rviz": "false",
                    }.items(),
                )
            ],
        ),
        # 3. Start Move Group
        TimerAction(
            period=move_group_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(move_group_launch),
                    launch_arguments={
                        "ur_type": ur_type,
                    }.items(),
                )
            ],
        ),
        # 4. Start RViz
        TimerAction(
            period=rviz_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_rviz_launch),
                    launch_arguments={
                        "ur_type": ur_type,
                        "launch_rviz": "true",
                        "rviz_config": rviz_config,
                    }.items(),
                )
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_delay", default_value="12"),
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
            DeclareLaunchArgument("rviz_config", default_value="moveit.rviz"),
            # 1. Start URSim
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ur_client_library",
                    "start_ursim.sh",
                    "-m",
                    "ur3e",
                    "-i",
                    "192.168.56.101",
                ],
                shell=True,
                output="screen",
            ),
            # 2–4. Delay launch with OpaqueFunction to compute timing
            OpaqueFunction(function=launch_setup),
        ]
    )
