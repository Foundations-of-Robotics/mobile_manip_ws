# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('headless', default_value='false',
                          choices=['true', 'false'], description='Start Gazebo headless.'),
    DeclareLaunchArgument('nogui', default_value='false',
                          choices=['true', 'false'], description='Start Gazebo without GUI.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('generate',
                          default_value='true',
                          choices=['true', 'false'],
                          description='Generate parameters and launch files'),
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.3',
                 description='z component of the robot pose.'))


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_mobile_manip = get_package_share_directory('mobile_manip')

    gz_sim_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'gz_sim.launch.py'])
    gz_sim_launch_nogui = PathJoinSubstitution([pkg_mobile_manip, 'launch', 'gz_sim_nogui.launch.py'])
    gz_sim_launch_headless = PathJoinSubstitution([pkg_mobile_manip, 'launch', 'gz_sim_headless.launch.py'])
    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    gz_sim = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gz_sim_launch]),
                launch_arguments=[('world', LaunchConfiguration('world'))],
                condition=UnlessCondition(LaunchConfiguration('headless'))
            )
        ],
        condition=UnlessCondition(LaunchConfiguration('nogui'))
    )
    gz_sim_ng = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_nogui]),
        launch_arguments=[('world', LaunchConfiguration('world'))],
        condition=IfCondition(LaunchConfiguration('nogui'))
    )
    gz_sim_hl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_headless]),
        launch_arguments=[('world', LaunchConfiguration('world'))],
        condition=IfCondition(LaunchConfiguration('headless'))
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', LaunchConfiguration('world')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            ('generate', LaunchConfiguration('generate'))]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_hl)
    ld.add_action(gz_sim_ng)
    ld.add_action(gz_sim)
    ld.add_action(robot_spawn)
    return ld
