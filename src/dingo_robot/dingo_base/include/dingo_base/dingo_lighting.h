/**
 *  \file       dingo_lighting.cpp
 *  \brief      Lighting control class for Dingo
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

#ifndef DINGO_BASE_DINGO_LIGHTING_H
#define DINGO_BASE_DINGO_LIGHTING_H

#include<vector>

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "dingo_msgs/Lights.h"
#include "dingo_msgs/Status.h"
#include "sensor_msgs/BatteryState.h"
#include "puma_motor_msgs/MultiStatus.h"

namespace dingo_base
{

typedef boost::array<uint32_t, 8> pattern;
typedef std::vector<pattern> LightsPatterns;

/** This class controls the 4 corner lights on Dingo. The lighting values
 *  can be based either on the current lighting state or on user-provided
 *  values.
 */
class DingoLighting
{
public:
  /** The set of states for which different lighting is provided */
  enum class State
  {
    Idle = 0,
    Driving,
    LowBattery,
    NeedsReset,
    Fault,
    Stopped
  };

  /** Initialize ROS communication, set up timers, and initialize lighting
   *  patterns for each state.
   *  @param[in] nh Handle used for ROS communication
   */
  explicit DingoLighting(ros::NodeHandle* nh);

private:
  /** Used for ROS communication */
  ros::NodeHandle* nh_;

  /** Used to publish corner light state to the MCU */
  ros::Publisher lights_pub_;

  /** Used to subscribe to user commands */
  ros::Subscriber user_cmds_sub_;

  /** Used to subscribe to MCU status */
  ros::Subscriber mcu_status_sub_;

  /** Used to subscrube to battery state */
  ros::Subscriber battery_state_sub_;

  /** Used to subscribe to Puma motor controller status */
  ros::Subscriber puma_status_sub_;

  /** Used to subscribe to velocity commands */
  ros::Subscriber cmd_vel_sub_;

  /** The last received Puma status message */
  puma_motor_msgs::MultiStatus pumas_status_msg_;

  /** The last received MCU status message */
  dingo_msgs::Status mcu_status_msg_;

  /** The last received battery state message */
  sensor_msgs::BatteryState battery_state_msg_;

  /** The last received velocity command */
  geometry_msgs::Twist cmd_vel_msg_;

  /** Used to trigger a periodic callback to update/publish state */
  ros::Timer pub_timer_;

  /** Used to track if lighting state has been published recently */
  ros::Timer user_timeout_;

  /** Tracks if currently in a lighting state where the user can override
   *  the lighting output
   */
  bool allow_user_;

  /** Used to track if the user is providing lighting values or if they are
   *  being generated internally based on the state
   */
  bool user_publishing_;

  /** The current lighting state */
  State state_;

  /** The previous lighting state */
  State old_state_;

  /** Tracks the current step in the output pattern */
  uint8_t current_pattern_count_;

  /** Currently lighting values being output */
  uint32_t current_pattern_[4];

  /** The lighting patterns to be used in the various lighting states */
  struct patterns
  {
    LightsPatterns stopped;
    LightsPatterns fault;
    LightsPatterns reset;
    LightsPatterns low_battery;
    LightsPatterns driving;
    LightsPatterns idle;
  }
  patterns_;

  /** Extract the RGB values for a single light.
   *  @param[out] rgb Stores the extract RGB values
   *  @param[in] colour Contains the encoded RGB values to be extracted
   */
  void setRGB(dingo_msgs::RGB* rgb, uint32_t colour);

  /** Sets the four corner lights from the given pattern.
   *  @param[out] lights Message to be updated with the lighting values
   *  @param[in] pattern Lighting pattern to be encoded into the message
   */
  void setLights(dingo_msgs::Lights* lights, uint32_t pattern[4]);

  /** Updates the current lighting state based on all inputs */
  void updateState();

  /** Updates the current lighting pattern based on the current state */
  void updatePattern();

  /** Called from a ROS message to update the lights. Updated values only
   *  get applied if not in an error or warning state.
   *  @param[in] lights_msg Updated lighting values to apply
   */
  void userCmdCallback(const dingo_msgs::Lights::ConstPtr& lights_msg);

  /** Called when a MCU status message is received, for state updates.
   *  @param[in] status_msg The status message to be stored
   */
  void mcuStatusCallback(const dingo_msgs::Status::ConstPtr& status_msg);

  /** Called when a battery state message is received, for state updates.
   *  @param[in] battery_msg The battery-state message to be stored
   */
  void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& battery_msg);

  /** Called when a velocity command is received, for state updates.
   *  @param[in] msg The velocity command to be stored
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /** Called when a Puma status message is received, for state updates.
   *  @param[in] status_msg The status message to be stored
   */
  void pumaStatusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg);

  /** Called periodically to update and publish the lighting pattern */
  void timerCallback(const ros::TimerEvent&);

  /** Called periodically to determine when the user has stopped sending
   *  lighting messages, indicating that state-based lighting should be used.
   */
  void userTimeoutCallback(const ros::TimerEvent&);
};

}  // namespace dingo_base

#endif  // DINGO_BASE_DINGO_LIGHTING_H
