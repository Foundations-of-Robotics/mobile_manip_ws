# mobile_manip_ros2

_Last updated: 16 january 2026_

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
sudo apt-get install -y build-essential bc flex bison libssl-dev zstd ros-humble-foxglove-bridge

sudo rosdep update && sudo rosdep install --from-paths src --ignore-src -y
sudo chown -R $(whoami) /home/ws/ && . /opt/ros/humble/setup.sh
src/build.sh
source install/setup.bash
```

## Simulation
Launch it with
```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=/home/ws/src world:=/home/ws/src/warehouse_duck
```
or to launch it altogether with foxglove bridge and (enventually) gen3 lite controller:
```bash
ros2 launch mobile_manip doody_sim.launch.py
```
