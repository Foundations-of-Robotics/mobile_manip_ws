cmake_minimum_required(VERSION 2.8.3)
project(mobile_manip)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  jackal_msgs
  controller_interface
  nav_msgs
  realtime_tools
  tf
  pluginlib
  gazebo_ros
  message_generation
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread random)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Declare the service files to be built
add_service_files(
   FILES
   ReachName.srv
   ReachValues.srv
   GetValues.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES mobile_manip
   CATKIN_DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Build plugins
add_library(gazebo_panel_plugin src/gazebo_plugins/gazebo_panel_plugin.cpp)
target_link_libraries(gazebo_panel_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Build nodes
add_library(dingo_custom_ros_controller src/dingo_custom_ros_controller.cpp src/odometry.cpp)
target_link_libraries(dingo_custom_ros_controller ${catkin_LIBRARIES})
add_dependencies(dingo_custom_ros_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
