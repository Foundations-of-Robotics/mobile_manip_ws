from launch import LaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue
import socket, fcntl, struct

def getifip(ifn):
	sck = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	if isinstance(ifn, str):
		ifn = ifn.encode('utf-8')
	return socket.inet_ntoa(fcntl.ioctl(sck.fileno(), 0x8915, struct.pack('256s', ifn[:15]))[20:24])

def get_internetIP():
	def _has_internet_via(local_ip, timeout=2.0):
		try:
			s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			s.settimeout(timeout)
			# bind to the interface's IP so the outgoing packet uses it
			s.bind((local_ip, 0))
			# connect to a reliable public DNS server
			s.connect(('8.8.8.8', 53))
			s.close()
			return True
		except Exception:
			return False

	# read /proc/net/dev and parse
	with open('/proc/net/dev', 'r') as f:
		lines = f.read().splitlines()

	# drop header lines if present
	if len(lines) >= 2:
		lines = lines[2:]

	for line in lines:
		if not line.strip():
			continue
		parts = line.split(':')
		if len(parts) != 2:
			continue
		ifname = parts[0].strip()
		try:
			local_ip = getifip(ifname)
		except Exception:
			continue
		if local_ip.startswith('127.'):
			continue
		if _has_internet_via(local_ip):
			#print(ifname)
			#print(local_ip)
			return local_ip

	return "0.0.0.0"



def generate_launch_description():
    myip = get_internetIP()
    print("My IP is: ", myip)

    headless = LaunchConfiguration('headless')
    robot_ip = LaunchConfiguration('robot_ip')
    
    # Get path to URDF
    urdf_path = PathJoinSubstitution([FindPackageShare('mobile_manip'), 'models/GEN3-LITE.urdf'])
    
    arm_ik_node = Node(
        package='mobile_manip',
        executable='arm_ik_controller',
        output='screen',
        parameters=[
            {
                'urdf_path': urdf_path,
                'pose_topic': '/mobile_manip/arm_command_node/ee_pose_cmd',
                'trajectory_topic': '/mobile_manip/arm_0_joint_trajectory_controller/joint_trajectory',
                'end_effector_frame': 'END_EFFECTOR',
                'base_frame': 'base_link',
                'trajectory_time': 5.0,
            }
        ]
    )
    
    mm_dir = FindPackageShare('mobile_manip')
    # Create the action to append the path to GZ_SIM_RESOURCE_PATH
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([mm_dir, "models/"])
    )
    
    launch_dir_mm = PathJoinSubstitution([mm_dir, 'launch'])
    dingo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([launch_dir_mm, 'simulation.launch.py'])]),
        launch_arguments={
            'headless': headless,
            'nogui': LaunchConfiguration('nogui'),
            'setup_path': PathJoinSubstitution([mm_dir,'config']),
            'rviz': 'false',
            #'world': PathJoinSubstitution([mm_dir,'worlds/productique']),
            #'world': PathJoinSubstitution([mm_dir,'worlds/warehouse_duck']),
            'world': PathJoinSubstitution([mm_dir,'worlds/A2230_map']), 'x': '-1.0', 'y': '1.0',
        }.items(),
    )
    # ---------------------------------------------------------------------------------------------.
    
    # Duplicates launch sequence for/without FoxGlove IP - should be some better way to do this..
    # 
    # ---------------------------------------------------------------------------------------------.
    launch_dir_fox = PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch'])
    foxglove_launch_hl = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution([launch_dir_fox, 'foxglove_bridge_launch.xml'])]),
        #XMLLaunchDescriptionSource([PathJoinSubstitution([mm_dir, 'launch/foxglove.xml'])]),
        launch_arguments={
            'address': myip,
        }.items(),
        condition=IfCondition(headless)
    )
    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution([launch_dir_fox, 'foxglove_bridge_launch.xml'])]),
        #XMLLaunchDescriptionSource([PathJoinSubstitution([mm_dir, 'launch/foxglove.xml'])]),
        condition=UnlessCondition(headless)
    )
    # ---------------------------------------------------------------------------------------------.
	
    device_launch = PathJoinSubstitution([launch_dir_mm, 'sensors_dummy.launch.py'])
    include_launch_sensors = IncludeLaunchDescription(PythonLaunchDescriptionSource([device_launch]))
    
    map_server_node = LaunchDescription([
            Node(
                package='nav2_map_server',
                executable='map_server',
                output='screen',
                parameters=[
                    {'frame_id': 'map'},
                    {'topic': 'map'},
                    {'use_sim_time': True},
                    {'yaml_filename': PathJoinSubstitution([mm_dir, 'maps/a2230_map_closed.yaml'])},
                ],
            )
        ])
    map_server_lc = LaunchDescription([
            Node(
                package='nav2_util',
                executable='lifecycle_bringup',
                output='screen',
                arguments=[
                    'map_server'
                ],
            )
        ])
    
    apriltag_ros = LaunchDescription([
            Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='apriltag',
                namespace='mobile_manip',
                output='screen',
                remappings=[('/tf', '/mobile_manip/tf')],
                parameters=[PathJoinSubstitution([mm_dir, 'config/tags.yaml'])],
                arguments=[
                    '--ros-args',
                    '-r', 'image_rect:=/mobile_manip/sensors/t265/fisheye1/image',
                    '-r', 'camera_info:=/mobile_manip/sensors/t265/fisheye1/camera_info',
                ],
            )
        ])
    
    tag_pose_node = LaunchDescription([
            Node(
                package='mobile_manip',
                namespace='mobile_manip',
                executable='tag_pose',
                output='screen',
                name='tag_pose_publisher',
                parameters=[
                    {'tf_topic': '/mobile_manip/tf'},
                    {'array_topic': '/mobile_manip/sensors/t265/tag_pose'}
                ]
            )
        ])

    fused_tf = LaunchDescription([
        Node(
            package='mobile_manip',
            namespace='mobile_manip',
            executable='fused_odom_tf',
            name='fused_odom_tf',
            output='screen',
            parameters=[
                {'fused_odom_topic': '/mobile_manip/sensors/fused_odometry'},
            ],
            remappings=[
                ('/tf', '/mobile_manip/tf'),
                ('/tf_static', '/mobile_manip/tf_static')
            ],
        ),
    ])

    odometry_path_publisher = LaunchDescription([
        Node(
            package='mobile_manip',
            namespace='mobile_manip',
            executable='odometry_path_publisher',
            name='odometry_path_publisher',
            output='screen',
            parameters=[
                {'scan_period_s': 10.0},
                {'path_buffer_size': 500},
                {'default_frame_id': 'map'},
            ],
        ),
    ])
    
    
    return LaunchDescription([
        DeclareLaunchArgument('nogui', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('robot_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        set_gz_resource_path,
        arm_ik_node,
        dingo_sim_launch,
        foxglove_launch,
        foxglove_launch_hl,
		include_launch_sensors,
        map_server_node,
        map_server_lc,
        apriltag_ros,
        tag_pose_node,
        fused_tf,
        odometry_path_publisher,
    ])
