from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    config_path = LaunchConfiguration('config_path')
    server_port = LaunchConfiguration('server_port')
    server_prefix = LaunchConfiguration('server_prefix')
    stop_ros_daemon = LaunchConfiguration('stop_ros_daemon')

    launch_dir_fox = PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch'])

    discovery_server_manager = Node(
        package='mobile_manip',
        executable='discovery_server_manager',
        output='screen',
        parameters=[
            {
                'robot_ip': robot_ip,
                'config_path': config_path,
                'server_port': server_port,
                'server_prefix': server_prefix,
                'stop_ros_daemon': stop_ros_daemon,
            }
        ],
    )
    
    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [PathJoinSubstitution([launch_dir_fox, 'foxglove_bridge_launch.xml'])]
        ),
    )

    tag_pose_node = Node(
        package='mobile_manip',
        namespace='mobile_manip',
        executable='tag_pose',
        output='screen',
        name='tag_pose_publisher',
        parameters=[
            {
                'tf_topic': '/mobile_manip/tf',
                'array_topic': '/mobile_manip/sensors/t265/tag_pose',
            }
        ],
    )

    fused_tf_node = Node(
        package='mobile_manip',
        namespace='mobile_manip',
        executable='fused_odom_tf',
        name='fused_odom_tf',
        output='screen',
        parameters=[
            {
                'fused_odom_topic': '/mobile_manip/sensors/fused_odometry',
            }
        ],
        remappings=[
            ('/tf', '/mobile_manip/tf'),
            ('/tf_static', '/mobile_manip/tf_static'),
        ],
    )

    odometry_path_publisher = Node(
        package='mobile_manip',
        namespace='mobile_manip',
        executable='odometry_path_publisher',
        name='odometry_path_publisher',
        output='screen',
        parameters=[
            {
                'scan_period_s': 10.0,
                'path_buffer_size': 500,
                'default_frame_id': 'map',
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip'),
        DeclareLaunchArgument('config_path', default_value='/tmp/super_client_generated.xml'),
        DeclareLaunchArgument('server_port', default_value='11811'),
        DeclareLaunchArgument(
            'server_prefix',
            default_value='44.53.00.5f.45.50.52.4f.53.49.4d.41',
        ),
        DeclareLaunchArgument('stop_ros_daemon', default_value='true'),
        discovery_server_manager,
        foxglove_launch,
        tag_pose_node,
        fused_tf_node,
        odometry_path_publisher,
    ])
