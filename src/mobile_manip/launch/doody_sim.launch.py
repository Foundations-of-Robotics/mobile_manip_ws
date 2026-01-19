from launch import LaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_dir_kortex = PathJoinSubstitution([FindPackageShare('kortex_bringup'), 'launch'])
    kortex_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([launch_dir_kortex, 'gen3_lite.launch.py'])]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware' : 'true',
            'launch_rviz': launch_rviz,
        }.items()
    )
    
    launch_dir_cpr = PathJoinSubstitution([FindPackageShare('doody_bringup'), 'launch'])
    mm_dir = FindPackageShare('mobile_manip')
    dingo_sim_launch_hl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([launch_dir_cpr, 'simulation.launch.py'])]),
        launch_arguments={
            'setup_path': PathJoinSubstitution([mm_dir,'config']),
            'rviz': launch_rviz,
            #'world': PathJoinSubstitution([mm_dir,'worlds/productique']),
            'world': PathJoinSubstitution([mm_dir,'worlds/warehouse_duck']),
            #'world': PathJoinSubstitution([mm_dir,'worlds/A_2230_v2']),
        }.items(),
        condition=IfCondition(headless)
    )
    
    launch_dir_cpr = PathJoinSubstitution([FindPackageShare('clearpath_gz'), 'launch'])
    mm_dir = FindPackageShare('mobile_manip')
    dingo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([launch_dir_cpr, 'simulation.launch.py'])]),
        launch_arguments={
            'setup_path': PathJoinSubstitution([mm_dir,'config']),
            'rviz': launch_rviz,
            #'world': PathJoinSubstitution([mm_dir,'worlds/productique']),
            'world': PathJoinSubstitution([mm_dir,'worlds/warehouse_duck']),
            #'world': PathJoinSubstitution([mm_dir,'worlds/A_2230_v2']),
        }.items(),
        condition=UnlessCondition(headless)
    )
    
    launch_dir_fox = PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch'])
    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution([launch_dir_fox, 'foxglove_bridge_launch.xml'])])
    )

    teleop = Node(
        package='mobile_manip',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
    )
    
    pkg_project_bringup = FindPackageShare('doody_bringup')
    device_launch = PathJoinSubstitution([pkg_project_bringup, 'launch', 't265.launch.py'])
    include_launch_t265 = IncludeLaunchDescription(PythonLaunchDescriptionSource([device_launch]))
    
    node_wheel_cmd = Node(
        name='wheel_vel',
        executable='wheel_vel',
        package='doody_bringup',
        namespace='mobile_manip/',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.81'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        #kortex_launch,
        #teleop,
        dingo_sim_launch,
        dingo_sim_launch_hl,
        foxglove_launch,
        include_launch_t265,
        node_wheel_cmd,
    ])
