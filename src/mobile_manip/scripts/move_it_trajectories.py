#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot
# then modified by David St-Onge to integrate severa mechanisms (i.e. services call, safety checks and moveit obstacles) to ease gen3 lite control

import sys
import time
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from mobile_manip.srv import ReachName,ReachValues,GetValues,GetValuesResponse
from math import pi
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import degrees, radians
from kortex_driver.srv import *
from kortex_driver.msg import *

class MoveItTrajectories(object):
  """MoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(MoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_it_trajectories')
    self.is_gripper_present = True

    try:
      gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
      self.gripper_joint_name = gripper_joint_names[0]
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      print(rospy.get_namespace(),self.degrees_of_freedom,self.is_gripper_present)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      print(self.robot.get_group_names())
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      rospy.sleep(2)

      gripper_group_name = "gripper"
      self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      print("Initializing node in namespace " + rospy.get_namespace())
      sn = rospy.Service('reach_name', ReachName, self.handle_reach_name)
      sj = rospy.Service('reach_joints', ReachValues, self.handle_reach_joints)
      sc = rospy.Service('reach_cartesian', ReachValues, self.handle_reach_cartesian)
      sg = rospy.Service('reach_gripper', ReachValues, self.handle_reach_gripper)
      sgc = rospy.Service('get_cartesian', GetValues, self.handle_get_cartesian)
      sgj = rospy.Service('get_joints', GetValues, self.handle_get_joints)


      # Init the action topic subscriber
      self.action_topic_sub = rospy.Subscriber(rospy.get_namespace() + "action_topic", ActionNotification, self.cb_action_topic)
      self.last_action_notif_type = None

      # Init the services
      clear_faults_full_name = rospy.get_namespace() + 'base/clear_faults'
      rospy.wait_for_service(clear_faults_full_name)
      self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)
      read_action_full_name = rospy.get_namespace() + 'base/read_action'
      rospy.wait_for_service(read_action_full_name)
      self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
      execute_action_full_name = rospy.get_namespace() + 'base/execute_action'
      rospy.wait_for_service(execute_action_full_name)
      self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)



      send_gripper_command_full_name = rospy.get_namespace() + '/base/send_gripper_command'
      rospy.wait_for_service(send_gripper_command_full_name)
      self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)


      #self.force_home_the_robot()
      self.reach_named_position("home")

    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

  def cb_action_topic(self, notif):
    self.last_action_notif_type = notif.action_event

  def do_clear_faults(self):
    try:
      self.clear_faults()
    except rospy.ServiceException:
      rospy.logerr("Failed to call ClearFaults")
      return False
    else:
      rospy.loginfo("Cleared the faults successfully")
      rospy.sleep(2.5)
      return True

  def wait_for_action_end_or_abort(self):
    while not rospy.is_shutdown():
      if (self.last_action_notif_type == ActionEvent.ACTION_END):
        rospy.loginfo("Received ACTION_END notification")
        return True
      elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
        rospy.loginfo("Received ACTION_ABORT notification")
        return False
      else:
        time.sleep(0.01)

  def force_home_the_robot(self):
    # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
    self.last_action_notif_type = None
    req = ReadActionRequest()
    req.input.identifier = 2
    try:
      res = self.read_action(req)
    except rospy.ServiceException:
      rospy.logerr("Failed to call ReadAction")
      return False
    # Execute the HOME action if we could read it
    else:
      # What we just read is the input of the ExecuteAction service
      req = ExecuteActionRequest()
      req.input = res.output
      rospy.loginfo("Sending the robot home...")
      try:
        self.execute_action(req)
      except rospy.ServiceException:
        rospy.logerr("Failed to call ExecuteAction")
        return False
      else:
        return self.wait_for_action_end_or_abort()


  def reach_named_position(self, target):
    arm_group = self.arm_group
    self.do_clear_faults()

    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)

    # if target == 'home':
    #   res = self.force_home_the_robot()
    # else:
    #Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    res = arm_group.execute(trajectory_message, wait=True)

    return res

  def reach_joint_angles(self, tolerance, joint_positions):
    arm_group = self.arm_group
    success = True

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    if self.degrees_of_freedom != len(joint_positions):
      return 0

    arm_group.set_joint_value_target(joint_positions)

    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: print(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def remove_virtual_wall(self, name):
    self.scene.remove_world_object(name)
    rospy.sleep(0.2)

  def add_virtual_wall(self, name, dimensions, pose):
    p = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = self.robot.get_planning_frame()
    pose.header.stamp = rospy.Time.now()
    self.scene.add_box(name, pose, (dimensions[0], dimensions[1], dimensions[2]))
    rospy.sleep(0.2)

  def reach_cartesian_pose(self, pose, tolerance, constraints, virtual_dist):
    arm_group = self.arm_group

    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)
    #arm_group.set_goal_orientation_tolerance(0.05)

    # Add a wall to the scene
    if(virtual_dist>0.05):
      yaw = math.atan2(pose.position.y,pose.position.x)
      q = quaternion_from_euler(0, 0, yaw)
      wall_pose = geometry_msgs.msg.PoseStamped()
      wall_pose.pose.position.x = pose.position.x + virtual_dist*math.cos(yaw)
      wall_pose.pose.position.y = pose.position.y + virtual_dist*math.sin(yaw)
      wall_pose.pose.position.z = 0.5
      wall_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)
      self.add_virtual_wall("wall", [0.2, 1.5, 1.5], wall_pose)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    arm_group.set_planning_time(45)

    # Set the cartesian goal
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    success = arm_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    arm_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    arm_group.clear_pose_targets()
    return success

  # def reach_gripper_position(self, relative_position):
  #   gripper_group = self.gripper_group

  #   # We only have to move this joint because all others are mimic!
  #   gripper_joint = self.robot.get_joint(self.gripper_joint_name)
  #   gripper_max_absolute_pos = gripper_joint.max_bound()
  #   gripper_min_absolute_pos = gripper_joint.min_bound()
  #   try:
  #     val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
  #     return val
  #   except:
  #     return False

  def reach_gripper_position(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

  def handle_reach_name(self, req):
    print("Calling handle_reach_name with", req)
    return self.reach_named_position(req.name)

  def handle_reach_joints(self, req):
    print("Calling handle_reach_joints with", req)
    return self.reach_joint_angles(tolerance=0.01, joint_positions=radians(req.val))

  def handle_reach_gripper(self, req):
    print("Calling handle_reach_gripper with", req)
    return self.reach_gripper_position(req.val[0])

  def handle_reach_cartesian(self, req):
    print("Calling handle_reach_cartesian with", req)
    desp = geometry_msgs.msg.Pose()
    desp.position.x = req.val[0]
    desp.position.y = req.val[1]
    desp.position.z = req.val[2]
    q = quaternion_from_euler(radians(req.val[3]), radians(req.val[4]), radians(req.val[5]))
    desp.orientation = geometry_msgs.msg.Quaternion(*q)
    return self.reach_cartesian_pose(pose=desp, tolerance=0.01, constraints=None, virtual_dist=req.val[6])

  def handle_get_cartesian(self, req):
    pose = self.get_cartesian_pose()
    rpy = degrees(euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
    return GetValuesResponse([pose.position.x, pose.position.y, pose.position.z, rpy[0], rpy[1], rpy[2]])

  def handle_get_joints(self, req):
    return GetValuesResponse(degrees(self.arm_group.get_current_joint_values()))

def main():
  runit = MoveItTrajectories()

  # For testing purposes
  success = runit.is_init_success

  if success:

    rospy.loginfo("--- Gen3 Lite MoveIt is ready !! ---")
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      rate.sleep()

    if not success:
        rospy.logerr("--- MoveIt! trajectory encountered an error. ---")

if __name__ == '__main__':
  main()
