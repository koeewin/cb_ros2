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
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    carrierbot_prefix = get_package_share_directory('cb')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  carrierbot_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='CB.lua')
    load_map = LaunchConfiguration('load_map', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    rviz_config_dir = os.path.join(turtlebot3_cartographer_prefix, 'rviz', 'tb3_cartographer.rviz')
    occupancy_grid_launch_file = os.path.join(turtlebot3_cartographer_prefix, 'launch', 'occupancy_grid.launch.py')

    def cartographer_node(context: LaunchContext):
        arguments_cartographer = ['-configuration_directory', cartographer_config_dir,
                                '-configuration_basename', configuration_basename]
        
        load_map_value = context.perform_substitution(load_map)
        if load_map_value.lower() != 'false':
            arguments_cartographer += ['-load_state_filename', load_map_value]
        
        return[
            Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=arguments_cartographer),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'load_map',
            default_value=load_map,
            description='path to pbstream-file with map'),

        OpaqueFunction(function=cartographer_node),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(occupancy_grid_launch_file),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

    ])
