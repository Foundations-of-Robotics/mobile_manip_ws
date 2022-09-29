# Dingo Setup

This file explains the necessary steps to configure the Dingos to be used for the Mobile Manipulator Lab. These commands must be run as the `administrator` user.

## LibRealsense Installation

These are the steps to install `librealsense2` on the Nvdia Jetson NX computer, required by the Realsense cameras. For more details see [this guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md).

1- Register the server's public key:

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```

> In case the public key cannot be retrieved, check and specify proxy settings: `export http_proxy="http://<proxy>:<port>"`, and rerun the command. See additional methods in the following [link](https://unix.stackexchange.com/questions/361213/unable-to-add-gpg-key-with-apt-key-behind-a-proxy).

2- Add the server to the list of repositories:

```
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```

3- Install the SDK:

```
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```

## ROS Workspace Installation

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
sudo apt install python3-rosdep ros-noetic-apriltag* ros-noetic-teleop-twist-keyboard ros-noetic-jackal-msgs ros-noetic-depthimage-to-laserscan
```
If you never ran rosdep before (fresh ROS install), then you need to run these initialization commands:
```
sudo rosdep init
rosdep update
```

### Build this repository

Deploy this repository and its submodules:
```
git clone -b master https://github.com/Foundations-of-Robotics/mobile_manip_ws
cd mobile_manip_ws
rosdep install --from-paths src --ignore-src -y
source scripts/add_remotes.sh
```
Build it (onboard the real robot):
```
cd ~/mobile_manip_ws/scripts
./build_real.sh
```
> If you get a SSL error from conan not able to reach to Kinova app public repository, try upgrade your conan version with: `sudo python3 -m pip install --upgrade conan`. If this doesn't work, you can try Kinova's suggested fix: [cannot install ros_kortex thread](https://githubmemory.com/repo/Kinovarobotics/ros_kortex/issues/203).

Then source the new packages:
```
source ~/mobile_manip_ws/devel/setup.bash
```
The `source` command line can also be copied in your ~/.bashrc to make it permanent:
```
echo "source ~/mobile_manip_ws/devel/setup.bash" >> ~/.bashrc
```

## Network Configuration

Each Dingo has a unique IP address - `192.168.0.5X` - and hostname - `cpr-ets05-0X` - where `X` is that Dingo's ID.

1 - Add the following lines to the `/etc/hosts` file:

```
192.168.0.51    cpr-ets05-01
192.168.0.52    cpr-ets05-02
192.168.0.53    cpr-ets05-03
192.168.0.54    cpr-ets05-04
192.168.0.55    cpr-ets05-05
192.168.0.56    cpr-ets05-06
192.168.0.57    cpr-ets05-07
192.168.0.58    cpr-ets05-08
```

2 - Change the name in the `/etc/hostname` file to the Dingo's corresponding hostname.

3 - Add the following lines to the end of the `/home/mecbot/.bashrc` and `/home/administrator/.bashrc` files, replacing `X` with the Dingo's ID:

```
source /home/administrator/mobile_manip_ws/devel/setup.bash
export ROS_IP=192.168.0.5X
export ROS_MASTER_URI=http://192.168.0.5X:11311
```

## Startup Configuration

We use a custom startup launch file to make sure the ROS applications running on the Dingo are compatible with the programs used in the lab. The system-wide ROS setup file located in `/etc/ros/setup.bash` must be altered so the system can find our custom launch file. Remove the line

```
source /home/administrator/dingo_ws/install/setup.bash
```

And add at the end

```
source /home/administrator/mobile_manip_ws/devel/setup.bash
```

## Bluetooth Controller Pairing

Put the controller into pairing mode by pressing and holding the Share & PS buttons until the controller’s LED flashes rapidly in white. Then SSH into the robot and run

```
sudo ds4drv-pair
```

Once the pairing is complete you should be able to control the robot using your controller.


## Realsense cameras test
Run `roslaunch mobile_manip rs_d400_and_t265.launch` to start the cameras. You should be able to see the line `RealSense Node Is Up!` appearing twice in the output.
