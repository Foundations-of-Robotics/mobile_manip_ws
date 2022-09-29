#!/usr/bin/env python

# This Python script pauses the Gazebo physics

import rospy
import time
from std_srvs.srv import Empty

def physics():
    # Sleep before unpausing Gazebo so the robot does not fall
    time.sleep(5)

    # Unpause the physics
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()

if __name__ == "__main__":
    physics()
