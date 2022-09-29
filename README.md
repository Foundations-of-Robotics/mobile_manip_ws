# Mobile Manipulator Lab Workspace

This repository contains the ROS packages required to deploy the workspace for the assessments in [Foundation of Robotics](https://foundations-of-robotics.org/) book. In an academic context, this workspace is not intended to be deployed by the students, but rather by the faculty employees in charge of the course. We recommend setting up several computer stations with this workspace on Linux Ubuntu accessible on-site and remotely (`ssh` terminal commands plus simulations visualization with GzWeb) by the students. The same workspace is used on-board the real robotic platforms. More information about their design and configuration can be found in the repository [Dingo-Gen3_lite]().


## Setup Overview
The following instructions will guide you to install and deploy simulators and control packages for the [Kinova Gen3 lite](https://www.kinovarobotics.com/en/products/robotic-arms/gen3-ultra-lightweight-robot) robotic arm and the [Clearpath Dingo UGV](https://clearpathrobotics.com/dingo-indoor-mobile-robot/) using ROS and Gazebo. Several packages are submodules added from third party contributors, and `mobile_manip` package contains all custom files specific to the course assignments.

The following short manuals are provided:
- [Install](doc/install.md) [essential]: general installation instructions to deploy and build the workspace sources.
- [Install GzWeb](doc/install_gzweb.md) [essential for remote work]: specific installation instructions to deploy and configure the Gazebo web (browser-based) visualization.
- [Install JupyterHub](doc/jupyterhub.md) [essential for remote work]: specific installation instructions to deploy and configure the the Jupyter Hub to run Python notebooks remotely.
- [AprilTag](doc/apriltag.md) [informational only]: specific instructions to change the April tags configuration (both for simulation and real robots).
- [SubTree usage](doc/subtree_usage.md)[informational only]: specific instruction how the use of the subtree (git submodules) structure.
- [Users management](scripts/README.md)[informational only]: several Python scripts to help with user (student) accounts management tailored to ÉTS infrastructure.


## Usage

### Gen3 lite alone simulation manipulator

To move the arm using MoveIt in Rviz, run:

`roslaunch mobile_manip gen3_lite_sim.launch` and `roslaunch mobile_manip rviz.launch`

In order to send a target movement, move the robot to the desired configuration inside RViz. On the Planning tab press "Plan and Execute" to complete it.

### Gen3 lite alone real manipulator

For moving the real Gen3 Lite robot with Rviz, launch :

`roslaunch mobile_manip gen3_lite_real.launch` and `roslaunch mobile_manip rviz.launch`

In order to send a target movement, move the robot to the desired configuration inside RViz. On the Planning tab press "Plan and Execute" to complete it.

### Dingo & Gen3 lite simulation

Launch the simulation with `roslaunch mobile_manip gen3_lite_dingo_labsim.launch`. You can control the dingo using the keyboard by running `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=mobile_manip/cmd_vel` and monitor the navigation stack performances with `roslaunch mobile_manip amcl_rviz.launch`.

### Dingo & Gen3 lite real robot

For joint control for both robots, launch `roslaunch mobile_manip gen3_lite_dingo_real.launch`. You can control the dingo using the keyboard by running `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=mobile_manip/cmd_vel` and monitor the navigation stack performances with `roslaunch mobile_manip amcl_rviz.launch`.
