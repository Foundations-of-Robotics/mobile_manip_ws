## Installation

### Dependencies
*tested with Ubuntu 20.04 and ROS Noetic*

1- Install ROS **full-desktop** properly. See the [tutorial](http://wiki.ros.org/melodic/Installation/Ubuntu). Do not forget `source /opt/ros/noetic/setup.bash` !

2- Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros) Most likely the ROS package should be sufficent:
```
sudo apt install ros-noetic-realsense-*
```

3- Install and configure conan (required for Kinova `ros_kortex`):
```
sudo apt install python3 python3-pip
sudo python3 -m pip install conan
sudo python3 -m pip install --upgrade requests
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++17 default
```

4- Install prerequisites
```
sudo apt install python3-rosdep ros-noetic-apriltag* ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-teleop-twist-keyboard ros-noetic-jackal-msgs ros-noetic-depthimage-to-laserscan
sudo pip install filterpy
```
If you never ran rosdep before (fresh ROS install), then you need to run these initialization commands:
```
sudo rosdep init
rosdep update
```

### Build this repository

Deploy this repository and its submodules:
```
git clone -b master https://git.initrobots.ca/Nerea/mobile_manip_ws.git
cd mobile_manip_ws
rosdep install --from-paths src --ignore-src -y
source scripts/add_remotes.sh
```
Build it for simulation:
```
cd ~/mobile_manip_ws
catkin_make
```
or onboard the real robot:
```
cd ~/mobile_manip_ws/scripts
./build_real.sh
```
Then source the new packages:
```
source devel/setup.bash
```
The `ros_kortex` Kinova driver is built on conan that detects and compiles it for right host configuration. On some hosts it may complain about a missing environment variable. You just need to set it and run catkin_make again:
```
export CONAN_REVISIONS_ENABLED=1
```
The `source` command line can also be copied in your ~/.bashrc to make it permanent:
```
echo "source ~/mobile_manip_ws/devel/setup.bash" >> ~/.bashrc
```


### Troubleshooting
1 - If you get an error about a missing or wrong REST variable from Gazebo (ignition), edit `~/.ignition/fuel/config.yaml` to replace `https://api.ignitionfuel.org` with `https://api.ignitionrobotics.org`

2- If you get an error about MarkupSafe upgrade pip with `sudo python3 -m pip install --upgrade pip` and then conan with `sudo python3 -m pip install --upgrade conan`.

3- If you can't connect to conan server, verify if bintray is in the conan remote with `conan remote list`, if it is remove it `conan remote remove conan-center` and add `conan remote add conancenter https://center.conan.io`.

4- If you get a SSL error from conan renew SSL with `conan config install https://github.com/conan-io/conanclientcert.git` (from [cannot install ros_kortex thread](https://github.com/Kinovarobotics/ros_kortex/issues/203)).
