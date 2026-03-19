from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    mm_dir = FindPackageShare('mobile_manip')
    
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
    
    robot_desc = "/home/ws/install/mobile_manip/share/mobile_manip/config/robot.urdf.xacro"
    xacro_file = FindPackageShare('mobile_manip').find('mobile_manip') + '/config/robot.urdf.xacro'
    robot_description = ParameterValue(Command(['xacro ', xacro_file]),value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace = "mobile_manip",
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ]
    )
    
    return LaunchDescription([robot_state_publisher])
