from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    ld = LaunchDescription()

    download_and_run_binary = Node(
        package="unity_binary",
        executable="download_and_run_binary",
        name="download_and_run_binary",
        output="log"
    )
    ld.add_action(download_and_run_binary)

    # teleop_twist_keyboard = Node(
    #     package="teleop_twist_keyboard",
    #     executable="teleop_twist_keyboard",
    #     name="teleop_twist_keyboard",
    #     output="log"
    # )
    # ld.add_action(teleop_twist_keyboard)

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("unity_launch"), "rviz", "unity.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    ld.add_action(rviz_node)

    json_tf_bridge_node = Node(
        package="json_tf_bridge",
        executable="json_tf_bridge_node",
        name="json_tf_bridge_node",
        output="log"
    )
    ld.add_action(json_tf_bridge_node)

    path = get_package_share_directory('pioneer_description')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path, 'launch', 'robot_state_publisher.launch.py'))))

    path = get_package_share_directory('ros_tcp_endpoint')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path, 'launch', 'endpoint.launch.py'))))

    return ld
