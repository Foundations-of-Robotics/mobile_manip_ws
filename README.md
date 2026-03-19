# mobile_manip_ros2

_Last updated: March 19 2026_

This repository documents how to set up the simulation stack for ÉTS robotic courses. It was tested on Ubuntu 22, and deployed for ROS 2 Humble and Gazebo Fortress.

## Installation
```bash
mkdir mobile_manip_ws && cd mobile_manip_ws
git clone <this repo>
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get update && apt-get install wget python3-pip -y
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress -y

sudo apt-get update && sudo apt-get install ros-humble-clearpath-simulator -y
sudo apt-get install -y build-essential bc flex bison libssl-dev zstd
sudo apt-get install -y  ros-humble-foxglove-bridge ros-humble-rmf-building-map-tools ros-humble-rmf-dev ros-humble-nav2-map-server

src/build.sh
source install/setup.bash
```

## Simulation
Launch it with
```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=<mobile_manip_ws>/src world:=<mobile_manip_ws>/src/warehouse_duck
```
or to launch it altogether with foxglove bridge and a custom Kinova Gen3 lite controller:
```bash
ros2 launch mobile_manip doody_sim_p3.launch.py
```
this launch file also accepts two parameters : `headless`, if set to false doesn't launch gazebo windows; `nogui`, if set to false doesn't launch the defult teleop panel of Clearpath simulation.