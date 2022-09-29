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

#include "dingo_base/dingo_lighting.h"
#include "dingo_base/dingo_power_levels.h"
#include <boost/assign/list_of.hpp>

namespace dingo_base
{

/** Contains the colour definitions for the corner lights */
namespace Colours
{
  static const uint32_t Off = 0x000000;
  static const uint32_t Red_H = 0xFF0000;
  static const uint32_t Red_M = 0xAA00000;
  static const uint32_t Red_L = 0x550000;
  static const uint32_t Green_H = 0x00FF00;
  static const uint32_t Green_M = 0x00AA00;
  static const uint32_t Green_L = 0x005500;
  static const uint32_t Blue_H = 0x0000FF;
  static const uint32_t Blue_M = 0x0000AA;
  static const uint32_t Blue_L = 0x000055;
  static const uint32_t Yellow_H = 0xFFFF00;
  static const uint32_t Yellow_M = 0xAAAA00;
  static const uint32_t Yellow_L = 0x555500;
  static const uint32_t Orange_H = 0xFFAE00;
  static const uint32_t Orange_M = 0xF0A80E;
  static const uint32_t Orange_L = 0xD69F27;
  static const uint32_t White_H = 0xFFFFFF;
  static const uint32_t White_M = 0xAAAAAA;
  static const uint32_t White_L = 0x555555;
}  // namespace Colours

DingoLighting::DingoLighting(ros::NodeHandle* nh) :
  nh_(nh),
  allow_user_(false),
  user_publishing_(false),
  state_(State::Idle),
  current_pattern_count_(0)
{
  lights_pub_ = nh_->advertise<dingo_msgs::Lights>("mcu/cmd_lights", 1);

  user_cmds_sub_ = nh_->subscribe("cmd_lights", 1, &DingoLighting::userCmdCallback, this);
  mcu_status_sub_ = nh_->subscribe("mcu/status", 1, &DingoLighting::mcuStatusCallback, this);
  battery_state_sub_ = nh_->subscribe("battery/status", 1, &DingoLighting::batteryStateCallback, this);
  puma_status_sub_ = nh_->subscribe("status", 1, &DingoLighting::pumaStatusCallback, this);
  cmd_vel_sub_ = nh_->subscribe("cmd_vel", 1, &DingoLighting::cmdVelCallback, this);

  pub_timer_ = nh_->createTimer(ros::Duration(1.0/5), &DingoLighting::timerCallback, this);
  user_timeout_ = nh_->createTimer(ros::Duration(1.0/1), &DingoLighting::userTimeoutCallback, this);

  using namespace Colours;  // NOLINT(build/namespaces)
  patterns_.stopped.push_back(boost::assign::list_of(Red_H)(Red_H)(Red_H)(Red_H));
  patterns_.stopped.push_back(boost::assign::list_of(Off)(Off)(Off)(Off));

  patterns_.fault.push_back(boost::assign::list_of(Orange_H)(Orange_H)(Orange_H)(Orange_H));
  patterns_.fault.push_back(boost::assign::list_of(Off)(Off)(Off)(Off));

  patterns_.reset.push_back(boost::assign::list_of(Off)(Red_H)(Off)(Red_H));
  patterns_.reset.push_back(boost::assign::list_of(Red_H)(Off)(Red_H)(Off));

  patterns_.low_battery.push_back(boost::assign::list_of(Orange_L)(Orange_L)(Orange_L)(Orange_L));
  patterns_.low_battery.push_back(boost::assign::list_of(Orange_M)(Orange_M)(Orange_M)(Orange_M));
  patterns_.low_battery.push_back(boost::assign::list_of(Orange_H)(Orange_H)(Orange_H)(Orange_H));
  patterns_.low_battery.push_back(boost::assign::list_of(Orange_M)(Orange_M)(Orange_M)(Orange_M));
  patterns_.low_battery.push_back(boost::assign::list_of(Orange_L)(Orange_L)(Orange_L)(Orange_L));

  patterns_.driving.push_back(boost::assign::list_of(Red_M)(White_M)(White_M)(Red_M));

  patterns_.idle.push_back(boost::assign::list_of(Red_L)(White_L)(White_L)(Red_L));
}

void DingoLighting::setRGB(dingo_msgs::RGB* rgb, uint32_t colour)
{
  rgb->red = static_cast<uint8_t>((colour & 0xFF0000) >> 16);
  rgb->green = static_cast<uint8_t>((colour & 0x00FF00) >> 8);
  rgb->blue = static_cast<uint8_t>((colour & 0x0000FF));
}

void DingoLighting::setLights(dingo_msgs::Lights* lights, uint32_t pattern[4])
{
  for (int i = 0; i < 4; i++)
  {
    setRGB(&lights->lights[i], pattern[i]);
  }
}

void DingoLighting::userCmdCallback(const dingo_msgs::Lights::ConstPtr& lights_msg)
{
  // If lighting output is controlled by user (not current state), apply it here
  if (allow_user_)
  {
    lights_pub_.publish(lights_msg);
  }
  user_publishing_ = true;
}

void DingoLighting::mcuStatusCallback(const dingo_msgs::Status::ConstPtr& status_msg)
{
  mcu_status_msg_ = *status_msg;
}

void DingoLighting::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& battery_msg)
{
  battery_state_msg_ = *battery_msg;
}

void DingoLighting::pumaStatusCallback(const puma_motor_msgs::MultiStatus::ConstPtr& status_msg)
{
  pumas_status_msg_ = *status_msg;
}

void DingoLighting::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_msg_ = *msg;
}

void DingoLighting::timerCallback(const ros::TimerEvent&)
{
  updateState();

  switch (state_)
  {
    case State::Idle:
    case State::Driving:
      allow_user_ = true;
      break;
    case State::LowBattery:
    case State::NeedsReset:
    case State::Fault:
    case State::Stopped:
      allow_user_ = false;
      break;
  }

  // If lighting output is controlled by current state, apply it here
  if (!user_publishing_ || !allow_user_)
  {
    dingo_msgs::Lights lights_msg;
    updatePattern();
    setLights(&lights_msg, &current_pattern_[0]);
    lights_pub_.publish(lights_msg);
  }
}

void DingoLighting::userTimeoutCallback(const ros::TimerEvent&)
{
  user_publishing_ = false;
}

void DingoLighting::updateState()
{
  if (mcu_status_msg_.stop_engaged == true)
  {
    state_ = State::Stopped;
  }
  else if (mcu_status_msg_.drivers_active == false)
  {
    state_ = State::NeedsReset;
  }
  else if (pumas_status_msg_.drivers.size() == 2 &&  // Dingo-D
          (pumas_status_msg_.drivers[0].fault != 0 ||
           pumas_status_msg_.drivers[1].fault != 0))
  {
    state_ = State::Fault;
  }
  else if (pumas_status_msg_.drivers.size() == 4 &&  // Dingo-O
          (pumas_status_msg_.drivers[0].fault != 0 ||
           pumas_status_msg_.drivers[1].fault != 0 ||
           pumas_status_msg_.drivers[2].fault != 0 ||
           pumas_status_msg_.drivers[3].fault != 0))
  {
    state_ = State::Fault;
  }
  else if (battery_state_msg_.power_supply_technology == sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN &&
           mcu_status_msg_.measured_battery <= dingo_power::BATTERY_SLA_LOW_VOLT )  // SLA battery & voltage is low
  {
    state_ = State::LowBattery;
  }
  else if (battery_state_msg_.power_supply_technology == sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN &&
           mcu_status_msg_.measured_battery >= dingo_power::BATTERY_SLA_OVER_VOLT )  // SLA battery & voltage over-volt
  {
    state_ = State::Fault;
  }
  else if (battery_state_msg_.power_supply_technology != sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN &&
           battery_state_msg_.percentage <= dingo_power::BATTERY_LITHIUM_LOW_PERCENT )  // Li battery & charge is low
  {
    state_ = State::LowBattery;
  }
  else if (battery_state_msg_.power_supply_technology != sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN &&
           battery_state_msg_.voltage >= dingo_power::BATTERY_LITHIUM_OVER_VOLT )  // Li battery & voltage over-volt
  {
    state_ = State::Fault;
  }
  else if (cmd_vel_msg_.linear.x != 0.0 ||
           cmd_vel_msg_.linear.y != 0.0 ||
           cmd_vel_msg_.angular.z != 0.0)
  {
    state_ = State::Driving;
  }
  else
  {
    state_ = State::Idle;
  }
}

void DingoLighting::updatePattern()
{
  if (old_state_ != state_)
  {
    current_pattern_count_ = 0;
  }

  switch (state_)
  {
    case State::Stopped:
      if (current_pattern_count_ >= patterns_.stopped.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.stopped[current_pattern_count_], sizeof(current_pattern_));
      break;
    case State::Fault:
      if (current_pattern_count_ >= patterns_.fault.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.fault[current_pattern_count_], sizeof(current_pattern_));
      break;
    case State::NeedsReset:
      if (current_pattern_count_ >= patterns_.reset.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.reset[current_pattern_count_], sizeof(current_pattern_));
      break;
    case State::LowBattery:
      if (current_pattern_count_ >= patterns_.low_battery.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.low_battery[current_pattern_count_], sizeof(current_pattern_));
      break;
    case State::Driving:
      if (current_pattern_count_ >= patterns_.driving.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.driving[current_pattern_count_], sizeof(current_pattern_));
      break;
    case State::Idle:
      if (current_pattern_count_ >= patterns_.idle.size())
      {
        current_pattern_count_ = 0;
      }
      memcpy(&current_pattern_, &patterns_.idle[current_pattern_count_], sizeof(current_pattern_));
      break;
  }
  old_state_ = state_;
  current_pattern_count_++;
}

}  // namespace dingo_base
