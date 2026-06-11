from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ###Controller###
    robot = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('igus_rebel'),'launch','rebel.launch.py'])]))

    ###Move Group MoveIt###
    moveit = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('igus_rebel_moveit_config'), 'launch','igus_rebel_motion_planner.launch.py'])])) 
    
    return LaunchDescription([controller,moveit])
