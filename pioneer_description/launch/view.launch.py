from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("pioneer_description"), "rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    ld = LaunchDescription()
    path = get_package_share_directory('pioneer_description')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path, 'launch', 'robot_state_publisher.launch.py'))))
    ld.add_action(rviz_node)

    return ld
