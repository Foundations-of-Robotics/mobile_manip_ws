cmake_minimum_required(VERSION 3.8)
project(mobile_manip)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

# generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseStampedArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(tag_pose src/tag_pose.cpp)
ament_target_dependencies(tag_pose rclcpp geometry_msgs tf2_msgs)
target_link_libraries(tag_pose "${cpp_typesupport_target}")

add_executable(odometry_path_publisher src/odometry_path_publisher.cpp)
ament_target_dependencies(odometry_path_publisher rclcpp geometry_msgs nav_msgs)

add_executable(fused_odom_tf src/fused_odom_tf.cpp)
ament_target_dependencies(fused_odom_tf rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(t265_odom src/t265_odom.cpp)
ament_target_dependencies(t265_odom rclcpp geometry_msgs nav_msgs)

add_executable(uwb_pose src/uwb_pose.cpp)
ament_target_dependencies(uwb_pose rclcpp geometry_msgs nav_msgs)

add_executable(wheel_vel src/wheel_vel.cpp)
ament_target_dependencies(wheel_vel rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  tag_pose
  odometry_path_publisher
  fused_odom_tf
  t265_odom
  uwb_pose
  wheel_vel
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gen3_lite_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_node
)

install(PROGRAMS
  scripts/arm_ik_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_ik_controller
)

install(PROGRAMS
  scripts/discovery_server_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME discovery_server_manager
)

install(
  DIRECTORY
    config
    launch
    models
    worlds
    maps    
  DESTINATION share/${PROJECT_NAME}/
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
