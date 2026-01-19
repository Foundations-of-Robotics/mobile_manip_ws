from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    kortex_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('kortex_bringup'),
            '/launch/gen3_lite.launch.py'
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'launch_rviz': launch_rviz,
        }.items()
    )

    teleop = Node(
        package='mobile_manip',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.10'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        kortex_launch,
        teleop,
    ])
