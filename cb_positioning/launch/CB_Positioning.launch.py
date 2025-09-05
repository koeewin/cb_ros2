from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cb_positioning',
            executable='vision_tasks',   # from setup.py console_scripts
            name='vision_tasks',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='cb_positioning',
            executable='publish_fusion',
            name='publish_fusion',
            output='screen',
        ),
        # If you launch other packages’ nodes (e.g., rviz2), add them here
        # Node(package='rviz2', executable='rviz2', arguments=['-d', '<path_to>.rviz']),

         # Node from cb (your other package)
        Node(
            package='cb',
            executable='serial_reader',
        ),
    ])
