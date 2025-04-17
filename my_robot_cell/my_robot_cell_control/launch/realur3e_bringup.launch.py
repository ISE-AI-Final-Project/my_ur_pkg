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
    rviz_config = LaunchConfiguration("rviz_config").perform(context)

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
        # 1. Start UR ROS Driver
        TimerAction(
            period=0.0,
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
        # 2. Start Move Group
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(move_group_launch),
                    launch_arguments={
                        "ur_type": ur_type,
                    }.items(),
                )
            ],
        ),
        # 3. Start RViz
        TimerAction(
            period=8.0,
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
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            DeclareLaunchArgument("robot_ip", default_value="10.10.0.61"),
            DeclareLaunchArgument("rviz_config", default_value="main.rviz"),
            OpaqueFunction(function=launch_setup),
        ]
    )
