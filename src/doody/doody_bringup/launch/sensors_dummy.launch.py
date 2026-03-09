from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Nodes
    node_t265_gz_bridge = Node(
        name='t265_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='/mobile_manip/sensors/',
        output='screen',
        parameters=
            [{'use_sim_time': True,
            'config_file': PathJoinSubstitution([FindPackageShare('doody_bringup'), 'config/t265.yaml']),
            },],
    )
    
    node_depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[('tf', '/mobile_manip/tf'),
                    ('scan', '/mobile_manip/sensors/camera_0/scan'),
                    ('depth', '/mobile_manip/sensors/camera_0/depth/image'),
                    ('depth_camera_info', '/mobile_manip/sensors/camera_0/depth/camera_info')],
        parameters=[PathJoinSubstitution([FindPackageShare('doody_bringup'), 'config/depthtoscan.yaml'])]
    )

    node_t265_gz_image_bridge = Node(
        name='t265_gz_image_bridge',
        executable='image_bridge',
        package='ros_gz_image',
        output='screen',
        arguments=['/mobile_manip/sensors/t265/fisheye1/image'],
        parameters=[{'use_sim_time': True}],
    )
    
    map_to_odom_tf = LaunchDescription([
        Node(package = "tf2_ros", 
                namespace = "mobile_manip",
                executable = "static_transform_publisher",
                remappings=[('tf', '/mobile_manip/tf')],
                arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"])
    ])
    
    node_t265_odom = Node(
        name='t265_odom',
        executable='t265_odom',
        package='doody_bringup',
        namespace='/mobile_manip/',
        remappings=[('tf', '/mobile_manip/tf')],
        output='screen',
    )
    
    node_uwb_pose = Node(
        name='uwb_pose',
        executable='uwb_pose',
        package='doody_bringup',
        remappings=[('tf', '/mobile_manip/tf')],
        namespace='/mobile_manip/',
        output='screen',
    )
    
    
    node_wheel_cmd = Node(
        name='wheel_vel',
        executable='wheel_vel',
        package='doody_bringup',
        remappings=[('tf', '/mobile_manip/tf')],
        namespace='/mobile_manip/',
        output='screen',
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(node_t265_gz_bridge)
    #ld.add_action(node_camera_0_static_tf)
    ld.add_action(node_t265_gz_image_bridge)
    ld.add_action(node_t265_odom)
    ld.add_action(node_uwb_pose)
    ld.add_action(node_wheel_cmd)
    ld.add_action(node_depth_to_laserscan)
    ld.add_action(map_to_odom_tf)
    return ld
