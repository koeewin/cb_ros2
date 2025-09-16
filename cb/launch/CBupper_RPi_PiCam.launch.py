from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
    
        Node(
            package="cb",
            executable="serial_reader",
        ),
        
        Node(
            package="cb",
            executable="led_service_node",
        ),

        Node(
            package="vision_tasks",
            executable="led_service_node",
        ),

        Node(
            package="vision_tasks",
            executable="publish_fusion",
        ),

    ])
