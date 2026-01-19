from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package Directory
    pkg_project_bringup = FindPackageShare('doody_bringup')

    # Launch File
    device_launch = PathJoinSubstitution([pkg_project_bringup, 'launch', 't265.launch.py'])

    # Include Launch
    include_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([device_launch]))
    
    # Launch a node   
    node_t265_odom = Node(
        name='t265_odom',
        executable='t265_odom',
        package='doody_bringup',
        namespace='mobile_manip/',
        output='screen',
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(include_launch)
    ld.add_action(node_t265_odom)
    return ld