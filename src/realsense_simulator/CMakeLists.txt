cmake_minimum_required(VERSION 3.0.2)
project(realsense_simulator)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    image_transport
    camera_info_manager
    sensor_msgs
)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++17")

catkin_package(
    DEPENDS
        roscpp
        gazebo_ros
        image_transport
        camera_info_manager
        sensor_msgs
)

include_directories(
    include)
include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(d435_gazebo_plugin src/D435Plugin.cpp src/gazebo_ros_d435.cpp)
target_link_libraries(d435_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(d435_gazebo_plugin ${catkin_EXPORTED_TARGETS})

add_library(t265_gazebo_plugin src/gazebo_ros_t265.cpp)
target_link_libraries(t265_gazebo_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
    d435_gazebo_plugin
    t265_gazebo_plugin
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir include)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
