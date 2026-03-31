from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    node_t265_gz_bridge = Node(
        name='t265_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='/mobile_manip/sensors/',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': PathJoinSubstitution([FindPackageShare('mobile_manip'), 'config/t265.yaml']),
        }],
    )

    node_depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[('/tf', '/mobile_manip/tf'),
                    ('scan', '/mobile_manip/sensors/camera_0/scan'),
                    ('depth', '/mobile_manip/sensors/camera_0/depth/image'),
                    ('depth_camera_info', '/mobile_manip/sensors/camera_0/depth/camera_info')],
        parameters=[PathJoinSubstitution([FindPackageShare('mobile_manip'), 'config/depthtoscan.yaml'])]
    )

    node_t265_gz_image_bridge = Node(
        name='t265_gz_image_bridge',
        executable='image_bridge',
        package='ros_gz_image',
        output='screen',
        arguments=['/mobile_manip/sensors/t265/fisheye1/image'],
        parameters=[{'use_sim_time': True}],
    )

    node_t265_odom = Node(
        name='t265_odom',
        executable='t265_odom',
        package='mobile_manip',
        namespace='/mobile_manip/',
        remappings=[('/tf', '/mobile_manip/tf')],
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_rate': 50.0,
        }],
    )

    node_uwb_pose = Node(
        name='uwb_pose',
        executable='uwb_pose',
        package='mobile_manip',
        remappings=[('/tf', '/mobile_manip/tf')],
        namespace='/mobile_manip/',
        output='screen',
        parameters=[{
            'publish_rate': 20.0,
        }],
    )

    node_wheel_cmd = Node(
        name='wheel_vel',
        executable='wheel_vel',
        package='mobile_manip',
        remappings=[('/tf', '/mobile_manip/tf')],
        namespace='/mobile_manip/',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(node_t265_gz_bridge)
    ld.add_action(node_t265_gz_image_bridge)
    ld.add_action(node_t265_odom)
    ld.add_action(node_uwb_pose)
    ld.add_action(node_wheel_cmd)
    ld.add_action(node_depth_to_laserscan)
    return ld
