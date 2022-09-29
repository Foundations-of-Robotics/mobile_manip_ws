#!/bin/bash

export ROS_MASTER_URI=http://cpr-ets05-01:11311/
export ROS_IP=`ifconfig wlan0 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://'`

echo "ROS_IP is"
printenv ROS_IP
