#!/bin/bash

EXCLUDED_PACKAGES="ddynamic_reconfigure;realsense_gazebo_plugins"
export CONAN_REVISIONS_ENABLED=1
catkin_make -DCATKIN_BLACKLIST_PACKAGES=$EXCLUDED_PACKAGES --cmake-args -DCONAN_TARGET_PLATFORM=jetson
