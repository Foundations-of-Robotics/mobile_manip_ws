cmake_minimum_required(VERSION 3.8)
project(doody_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(PROGRAMS
  scripts/t265_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME t265_odom
)
install(PROGRAMS
  scripts/uwb_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME uwb_pose
)
install(PROGRAMS
  scripts/wheel_vel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_vel
)

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

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
