#!/bin/bash
# Setup script for the (mobile_manip_ws) environment
# Requires sudo privileges
#
# Usage: 
#   sudo ./setup_build.sh
# -----------------------------------------------------------------------------

# defining warning header
read -r -d '' warning_header << EOM
(Build) setup script for the (mobile_manip_ws) environment

SUDO PRIVILEGES AND A STABLE INTERNET CONNECTION ARE REQUIRED

WARNING
-------

DO NOT INTERRUPT EXECUTION ONCE THE SCRIPT IS STARTED.
DO NOT RE-RUN THIS SCRIPT ONCE THE ENVIRONMENT IS SET UP.

This script's execution time is around 20 minutes and it requires 16GB of disk space. 

It will perform the following : 
    1) Setup docker on the machine
    2) Build the (book_simulations) docker container in which all gazebo simulations will run
    3) Clone the mobile_manip_notebooks repository and place it next to the (mobile_manip_ws) folder
EOM

# displaying the script warning + handling user input
echo -e "\n$warning_header\n\n"
read -p "Type (yes) if you wish to continue, or anything else to abort : " warning_input
if [ $warning_input != "yes" ]; then
    echo -e "\n\nAborting execution.\n\n"
    exit
fi

echo -e "\n\nInstalling general dependencies ... \n\n"
apt-get install -y curl git

echo -e "\n\nCloning the (mobile_manip_notebooks) repository \n\n"
# check to prevent crushing (mobile_manip_notebooks) 
if [ -d "mobile_manip_notebooks" ]; then
    echo -e "\n\nThe (mobile_manip_notebooks) repository already exists - cloning aborted.\n\n"
else
    git clone https://github.com/Foundations-of-Robotics/mobile_manip_notebooks
    chmod -R 777 mobile_manip_notebooks/
fi

echo -e "\n\nCloning the (mobile_manip_ws) repository ... "
git clone https://github.com/Foundations-of-Robotics/mobile_manip_ws.git && cd mobile_manip_ws
chmod -R 777 docker/ && cd docker/setup/

echo -e "\n\nSetting up docker ... \n\n"

if [ -x "$(command -v docker)" ]; then
    echo -e "\n\nDocker is already installed.\n\n"
else
    
    apt-get remove -y docker docker-engine docker.io containerd runc
    curl -fsSL https://get.docker.com -o get-docker.sh
    sh get-docker.sh
    rm get-docker.sh
    sleep 5
fi

echo -e "\n\nBuilding the (book_simulations) docker container ...\n\n"

mkdir ./repo_files
cp -r ../../src repo_files/
cp -r ../../.git repo_files/
cp -r ../../scripts repo_files/
cp -r ../../.catkin_workspace repo_files/

chmod 777 -R repo_files/
docker build ./ -t book_simulations
rm -r repo_files/

echo -e "\n\nGiving non-root users acces to docker ...\n\n"

groupadd docker || true
usermod -aG docker $USER || true 
chmod g+rwx "$HOME/.docker" -R || true
chmod 666 /var/run/docker.sock || true

echo -e "\n\Setup complete .\n\n"