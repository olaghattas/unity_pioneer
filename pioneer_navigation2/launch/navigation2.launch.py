# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    slam = LaunchConfiguration('use_slam', default='False')

    # slam = LaunchConfiguration('slam')
    # declare_slam_cmd = DeclareLaunchArgument(
    #     'slam',
    #     default_value='False',
    #     description='Whether run a SLAM')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('pioneer_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = 'pioneer.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('pioneer_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load'))

    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load'))

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))

    # ld.add_action(DeclareLaunchArgument(
    #     'slam',
    #     default_value='True',
    #     description='Whether run a SLAM')
    # )

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir,
            'slam': slam}.items(),
    ))
    # IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([slam_launch_file_dir, '/online_async_launch.py']),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'slam_params_file': param_dir}.items(),
    # ),
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'))

    # ld2 = LaunchDescription()
    # ld.add_action(Node(
    #     package='nav2_waypoint_follower',
    #     executable='waypoint_follower',
    #     name='waypoint_follower',
    #     parameters=[param_dir],
    #     output='screen'))

    return ld