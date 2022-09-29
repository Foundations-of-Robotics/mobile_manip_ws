/**
 *  \file       dingo_cooling.h
 *  \brief      Cooling control class for Dingo
 *  \copyright  Copyright (c) 2020, Clearpath Robotics, Inc.
 *
 * Software License Agreement (BSD)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */

#ifndef DINGO_BASE_DINGO_COOLING_H
#define DINGO_BASE_DINGO_COOLING_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dingo_msgs/Status.h"
#include "dingo_msgs/Fans.h"

namespace dingo_base
{

/** This class manages cooling/fans for Dingo */
class DingoCooling
{
public:
  /** Set up publishers, subscribers, and initial fan state.
   *  @param[in] nh Handle used for ROS communication
   */
  explicit DingoCooling(ros::NodeHandle* nh);

private:
  /** Used for ROS communication */
  ros::NodeHandle* nh_;

  /** Used to publish fan state */
  ros::Publisher cmd_fans_pub_;

  /** Used to subscribe to velocity commands */
  ros::Subscriber cmd_vel_sub_;

  /** Timer for periodic callback to adjust fans */
  ros::Timer cmd_fans_timer_;

  /** Message to be sent to MCU to set fan level */
  dingo_msgs::Fans cmd_fans_msg_;

  /** Time of the last velocity command received */
  double last_motion_cmd_time_;

  /** Linear velocity threshold, in metres per second, which, if exceeded,
   *  will cause the fan to be turned on HIGH. Applies to linear velocity
   *  in either the x or y direction.
   */
  static const double LINEAR_VEL_THRESHOLD;
  /** Angular velocity threshold, in radians per second, which, if exceeded,
   *  will cause the fan to be turned on HIGH
   */
  static const double ANGULAR_VEL_THRESHOLD;
  /** The time, in seconds, after receiving the last velocity/motion command
   *  that the fans are set to LOW.
   */
  static const double MOTION_COMMAND_TIMEOUT;

  /** Used to adjust fans when certain velocity thresholds are exceeded
   *  @param[in] twist Details of the last velocity command
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist);

  /** Used to adjust fans periodically and publish to the MCU */
  void cmdFansCallback(const ros::TimerEvent&);
};

}  // namespace dingo_base

#endif  // DINGO_BASE_DINGO_COOLING_H
