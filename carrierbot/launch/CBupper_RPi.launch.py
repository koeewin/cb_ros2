from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package="carrierbot",
            executable="current_image",
        ),
       
        Node(
            package="carrierbot",
            executable="current_AprilTag",
        ),

        Node(
            package="carrierbot",
            executable="serial_reader",
        ),
    
    ])