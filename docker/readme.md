# Docker instructions

**This page details the steps required to set up the containerized mobile manipulator lab workspace.**

We made a docker image (book_simulations) containing everything needed to complete the exercises presented in the book. Once this image is installed on your machine, you will be able to interact with it, mainly via jupyter-notebok and gazebo web, to complete the exercises.

## Installing the book_simulations docker image

**The are two ways of getting the (book_simulations) docker image: downloading it or building it from scratch.** In both cases, the system requirements are: 
* Ubuntu 18.04 or greater
* Access to root privileges
* wget : `sudo apt install wget`

### Option 1 (Recommended) : Downloading the image

The steps to load the pre-built image are the following : 

1. Navigate to a folder you want to use as a working directory.

2. Download and execute the download script 
    - `wget https://raw.githubusercontent.com/Foundations-of-Robotics/mobile_manip_ws/master/docker/setup/setup_download.sh`
    - `sudo chmod +x setup_download.sh`
    - `sudo ./setup_download.sh`

### Option 2 : Building the image

The steps to build the image from scratch are the following :

1. Navigate to a folder you want to use as a working directory.

2. Download and execute the build script 
    - `wget https://raw.githubusercontent.com/Foundations-of-Robotics/mobile_manip_ws/master/docker/setup/setup_build.sh`
    - `sudo chmod +x setup_build.sh`
    - `sudo ./setup_build.sh`

## Using the book_simulations docker image

### Video demonstration

[![SIMULATION DEMO](https://i.ytimg.com/vi/1AwXbDujScw/maxresdefault.jpg)](https://www.youtube.com/watch?v=1AwXbDujScw&ab_channel=davidolivier)

All the robotic simulations are executed from inside the (book_simulations) container. When launched, the container starts **gzweb** (forwarded to port 8887) and **jupyter-notebook** (forwarded to port 8888).

### Starting / Stoping the book_simulations container

1. From your work directory, go to the folder containing the docker scripts
    - `cd mobile_manip_ws/docker`

2. Start or Stop the containers
    - To start the container : `./set_container_state.sh 1`
    - To stop the container : `./set_container_state.sh 0`

3. Once the container is running, you can acces jupyter-notebook by connecting to [http://localhost:8888/](http://localhost:8888/) via an internet browser.

**Note: you will have to use another terminal for the following steps.**

### Launching the simulations

There is a launch script available for all 4 simulations covered in the book. **Note that only one simulation can run at a time**. 

1. From your work directory, go to the folder containing the docker scripts
    - `cd mobile_manip_ws/docker`

2. Launch one of the simulation scripts
    * `./dingo_arenasim.sh`
    * `./gen3_lite_dingo_emptysim.sh`
    * `./gen3_lite_dingo_labsim.sh`
    * `./gen3_lite_sim.sh`

3. Once the simulation is running, you can acces **gzweb** by connecting to [http://localhost:8887/](http://localhost:8887/) via an internet browser.