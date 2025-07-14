from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package="cb",
            executable="current_image",
        ),
       
        Node(
            package="cb",
            executable="current_AprilTag",
        ),

        Node(
            package="cb",
            executable="serial_reader",
        ),
    
    ])
