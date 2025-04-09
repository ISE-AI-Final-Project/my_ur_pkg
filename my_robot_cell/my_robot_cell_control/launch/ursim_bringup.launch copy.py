import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type', default='ur3e')
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.56.101')

    # Paths to launch files
    cell_control_launch = os.path.join(
        get_package_share_directory('my_robot_cell_control'),
        'launch', 'start_robot.launch.py')

    move_group_launch = os.path.join(
        get_package_share_directory('my_robot_cell_moveit_config'),
        'launch', 'move_group.launch.py')

    moveit_rviz_launch = os.path.join(
        get_package_share_directory('my_robot_cell_moveit_config'),
        'launch', 'moveit_rviz.launch.py')

    return LaunchDescription([
        # 1. Start URSim script (in background)
        ExecuteProcess(
            cmd=['ros2', 'run', 'ur_client_library', 'start_ursim.sh', '-m', 'ur3e', '-i', '192.168.56.101'],
            shell=True,
            output='screen'
        ),

        # 2. Start UR ROS Driver (delayed to let URSim boot)
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cell_control_launch),
                    launch_arguments={
                        'ur_type': ur_type,
                        'robot_ip': robot_ip
                    }.items()
                )
            ]
        ),

        # 3. Start Move Group (delayed a bit more)
        TimerAction(
            period=16.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(move_group_launch),
                    launch_arguments={'ur_type': ur_type}.items()
                )
            ]
        ),

        # 4. Start MoveIt RViz
        TimerAction(
            period=18.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(moveit_rviz_launch),
                    launch_arguments={
                        'ur_type': ur_type,
                        'launch_rviz': 'true'
                    }.items()
                )
            ]
        )
    ])
