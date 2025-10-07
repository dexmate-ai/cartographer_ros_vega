"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'False')

    ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')

    ## ***** Nodes *****
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'rplidar_2d_online.lua',
            ],
        remappings = [
            ('imu', '/scan_imu'),
            ('points2', '/scan_pcd'),
            ('scan_matched_points2', '/registered_scan'),
            ],
        output = 'screen',
        # prefix=['gdbserver localhost:3000'],
        # prefix=['xterm -e gdb -ex run --args'],
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )
    
    laser_to_vehicle_tf_publisher = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'laser_link', 'sensor'],
        output = 'screen'
        )


    laser_to_base_tf_publisher = Node(
      package = 'tf2_ros',
      executable = 'static_transform_publisher',
      arguments = ['-0.1114', '0.0', '-0.0733', '0.0', '0.0', '0.0', 'laser_link', 'base_link'],
      output = 'screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        cartographer_node,
        cartographer_occupancy_grid_node,
        laser_to_vehicle_tf_publisher,
        laser_to_base_tf_publisher,
    ])
