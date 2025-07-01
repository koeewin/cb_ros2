from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import serial.tools.list_ports
import os

def generate_launch_description():

    carrierbot_prefix = get_package_share_directory('cb')
    rplidar_prefix = get_package_share_directory('rplidar_ros')

    cartographer_launch_file = os.path.join(carrierbot_prefix, 'launch', 'CBcartographer.launch.py')
    use_sim_time_value = 'false'

    rplidar_launch_file = os.path.join(rplidar_prefix, 'launch', 'rplidar_c1_launch.py')

    # Find out serial ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if '/dev/ttyUSB' in port.device:
            if 'UART' in port.description:
                serial_port_LIDAR = port.device
                


    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_file),
            launch_arguments={'use_sim_time': use_sim_time_value}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file),
            launch_arguments={'serial_port': serial_port_LIDAR}.items()
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0.067', '0.0', '0.4482', '0.0', '0.0', '1.0', '0.0', 'base_link', 'laser']
        ),
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0.135', '0.0', '0.424', '0.5', '-0.5', '0.5', '-0.5', 'base_link', 'cam_link']
        ),
        
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['-0.126', '0.0', '0.665', '0.0', '0.0', '0.0', '1.0', 'base_link', 'tracker']
        ),
            
        Node(
            package="diablo_ctrl",
            executable="diablo_ctrl_node",
        ),        

        Node(
            package="cb",
            executable="diablo_odometry",
        ),
       
        Node(
            package="cb",
            executable="motionctrl_diablo",
        ),

        Node(
            package="cb",
            executable="current_pose",
        ),
        
        Node(
            package="cb",
            executable="pp_control",
        ),
        
        Node(
            package="cb",
            executable="current_path_sim",
        ),


        Node(
            package="cb",
            executable="state_machine_sim",
        ),

       
    
    ])
