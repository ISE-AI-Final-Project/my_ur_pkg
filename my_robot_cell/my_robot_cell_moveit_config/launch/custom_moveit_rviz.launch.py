from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the launch argument for the RViz config filename
    declare_rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value='moveit.rviz',
        description='Name of the RViz config file located in the config directory'
    )

    # Construct the full path to the RViz config file
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_cell_moveit_config'),
        'config',
        LaunchConfiguration('rviz_config')
    ])

    # Define the RViz node with the specified config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path],
        parameters=[]  # Add necessary parameters here
    )

    return LaunchDescription([
        declare_rviz_config_arg,
        rviz_node
    ])
