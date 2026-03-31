from pathlib import Path
import getpass
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.actions import SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ROBOT_IP_PREFIX = '192.168.0.5'


def _build_xml(robot_ip: str, server_port: int, server_prefix: str) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="super_client_profile" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                        <discoveryServersList>
                            <RemoteServer prefix="{server_prefix}">
                                <metatrafficUnicastLocatorList>
                                    <locator>
                                        <udpv4>
                                            <address>{robot_ip}</address>
                                            <port>{server_port}</port>
                                        </udpv4>
                                    </locator>
                                </metatrafficUnicastLocatorList>
                            </RemoteServer>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
"""


def _robot_ip_from_id(robot_id: str) -> str:
    try:
        robot_number = int(robot_id)
    except ValueError as exc:
        raise ValueError('robot_id must be an integer from 1 to 8.') from exc

    if robot_number < 1 or robot_number > 8:
        raise ValueError('robot_id must be an integer from 1 to 8.')

    return f'{ROBOT_IP_PREFIX}{robot_number}'


def _ensure_parent_dir(path_value: str) -> None:
    Path(path_value).parent.mkdir(parents=True, exist_ok=True)


def _foxglove_port_from_username() -> str:
    username = getpass.getuser()
    match = re.fullmatch(r'mecbotg([0-9])', username)
    if match:
        return f'878{match.group(1)}'
    return '8765'


def _configure_discovery(context, *args, **kwargs):
    del args
    del kwargs

    robot_id = LaunchConfiguration('robot_id').perform(context)
    robot_ip = _robot_ip_from_id(robot_id)
    config_path = str(Path(LaunchConfiguration('config_path').perform(context)).expanduser())
    server_port = int(LaunchConfiguration('server_port').perform(context))
    server_prefix = LaunchConfiguration('server_prefix').perform(context)

    _ensure_parent_dir(config_path)
    Path(config_path).write_text(
        _build_xml(robot_ip, server_port, server_prefix),
        encoding='utf-8',
    )

    return [
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', config_path),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', config_path),
        UnsetEnvironmentVariable('ROS_DISCOVERY_SERVER'),
        UnsetEnvironmentVariable('ROS_LOCALHOST_ONLY'),
        LogInfo(
            msg=(
                f'Configured launch-scoped robot discovery for robot_id={robot_id} '
                f'at {robot_ip}:{server_port}'
            )
        ),
        LogInfo(msg=f'Fast DDS profile written to {config_path}'),
        LogInfo(
            msg='To access the same robot from another terminal, run: connect_robot '
            f'{robot_id}'
        ),
    ]


def generate_launch_description():
    launch_dir_fox = PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch'])
    foxglove_port = _foxglove_port_from_username()

    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [PathJoinSubstitution([launch_dir_fox, 'foxglove_bridge_launch.xml'])]
        ),
        launch_arguments={
            'port': foxglove_port,
        }.items(),
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
                'use_sim_time': True,
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
        DeclareLaunchArgument(
            'robot_id',
            description='Robot identifier from 1 to 8. Maps to 192.168.0.5X.',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value='~/.ros/mobile_manip/super_client_generated.xml',
        ),
        DeclareLaunchArgument('server_port', default_value='11811'),
        DeclareLaunchArgument(
            'server_prefix',
            default_value='44.53.00.5f.45.50.52.4f.53.49.4d.41',
        ),
        OpaqueFunction(function=_configure_discovery),
        LogInfo(msg=f'Foxglove port is: {foxglove_port}'),
        foxglove_launch,
        tag_pose_node,
        fused_tf_node,
        odometry_path_publisher,
    ])
