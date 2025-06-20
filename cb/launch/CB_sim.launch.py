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

    carrierbot_prefix = get_package_share_directory('carrierbot')
    turtlebot_gazebo_prefix = get_package_share_directory('turtlebot3_gazebo')
    cartographer_launch_file = os.path.join(carrierbot_prefix, 'launch', 'CBcartographer_sim.launch.py')
    
    map_file = LaunchConfiguration('map', default='false')

    use_sim_time_value = 'true'
    dir_parts_ws = get_package_prefix('carrierbot').split(os.sep)
    dir_ws = os.sep.join(dir_parts_ws[:-2])
    maps_path = os.path.join(dir_ws, 'src/carrierbot/map_files')

    turtlebot_gazebo_launch_file = os.path.join(turtlebot_gazebo_prefix, 'launch', 'turtlebot3_world.launch.py')

    if os.path.isdir(maps_path) == 0:
        print("\nERROR: Directory of map files doesn't exist!\n")
    maps_list = os.listdir(maps_path)
    if len(maps_list) == 0:
        print("\nERROR: No maps found!\n")
    max = 0
    for map_file_name in maps_list:
        map_time = os.path.getmtime(os.path.join(maps_path,map_file_name))
        if map_time > max:
            max = map_time
            map_newest = map_file_name

    def cartographer_launch(context: LaunchContext):
        map_value = context.perform_substitution(map_file)
        if map_value.lower() != 'false':
            if map_value.lower() == 'true':
                arguments = {'use_sim_time': use_sim_time_value,
                            'load_map': os.path.join(maps_path,map_newest)}.items()
            else:
                arguments = {'use_sim_time': use_sim_time_value,
                            'load_map': os.path.join(maps_path,map_value)}.items()
        else:
            arguments = {'use_sim_time': use_sim_time_value}.items()

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cartographer_launch_file),
                launch_arguments=arguments
            ),
        ]
    
    return LaunchDescription([

        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='decides weather old map should be used or not'),

        OpaqueFunction(function=cartographer_launch),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot_gazebo_launch_file),
        ), 

        Node(
            package="carrierbot",
            executable="current_pose",
        ), 

        Node(
            package="carrierbot",
            executable="current_path_sim",
        ), 

        Node(
            package="carrierbot",
            executable="motionctrl_sim",
        ), 


        Node(
            package="carrierbot",
            executable="pp_control",
        ), 

          Node(
            package="carrierbot",
            executable="state_machine_sim",
        ), 


    ])
