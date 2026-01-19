from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    launch_arg_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='')

    prefix = LaunchConfiguration('prefix')

    # Nodes
    node_t265_gz_bridge = Node(
        name='t265_gz_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='mobile_manip/sensors/',
        output='screen',
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                    'config_file': PathJoinSubstitution([FindPackageShare('doody_bringup'), 'config/t265.yaml'])
                    ,
                }
                ,
            ]
        ,
    )

    node_t265_gz_image_bridge = Node(
        name='t265_gz_image_bridge',
        executable='image_bridge',
        package='ros_gz_image',
        namespace='mobile_manip/sensors/',
        output='screen',
        arguments=
            [
                '/mobile_manip/sensors/t265/fisheye1/image'
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )
    
    node_t265_odom = Node(
        name='t265_odom',
        executable='t265_odom',
        package='doody_bringup',
        namespace='mobile_manip/',
        output='screen',
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_arg_prefix)
    ld.add_action(node_t265_gz_bridge)
    #ld.add_action(node_camera_0_static_tf)
    ld.add_action(node_t265_gz_image_bridge)
    ld.add_action(node_t265_odom)
    return ld
