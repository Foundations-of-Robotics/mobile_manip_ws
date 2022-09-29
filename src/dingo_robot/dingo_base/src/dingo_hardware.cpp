/**
 *  \file       dingo_hardware.cpp
 *  \brief      Class representing Dingo hardware
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

#include <boost/assign.hpp>
#include <vector>

#include <puma_motor_driver/driver.h>
#include <puma_motor_driver/multi_driver_node.h>

#include "dingo_base/dingo_hardware.h"

namespace dingo_base
{

DingoHardware::DingoHardware(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                             puma_motor_driver::Gateway& gateway, bool& dingo_omni):
  nh_(nh),
  pnh_(pnh),
  gateway_(gateway),
  active_(false)
{
  pnh_.param<double>("gear_ratio", gear_ratio_, 24);
  pnh_.param<int>("encoder_cpr", encoder_cpr_, 10);
  pnh_.param<bool>("flip_motor_direction", flip_motor_direction_, false);
  pnh_.param<double>("gains/p", gain_p_, 0.025);
  pnh_.param<double>("gains/i", gain_i_, 0.005);
  pnh_.param<double>("gains/d", gain_d_, 0.0);

  // Set up the wheels: differs for Dingo-D vs Dingo-O
  ros::V_string joint_names;
  std::vector<uint8_t> joint_can_ids;
  std::vector<float> joint_directions;
  if (!dingo_omni)
  {
    joint_names.assign({"left_wheel", "right_wheel"});  // NOLINT(whitespace/braces)
    joint_can_ids.assign({2, 3});                       // NOLINT(whitespace/braces)
    joint_directions.assign({1, -1});                   // NOLINT(whitespace/braces)
  }
  else
  {
    joint_names.assign({"front_left_wheel", "front_right_wheel",  // NOLINT(whitespace/braces)
        "rear_left_wheel", "rear_right_wheel"});                  // NOLINT(whitespace/braces)
    joint_can_ids.assign({2, 3, 4, 5});                           // NOLINT(whitespace/braces)
    joint_directions.assign({1, -1, 1, -1});                      // NOLINT(whitespace/braces)
  }

  // Flip the motor direction if needed
  if (flip_motor_direction_)
  {
    for (std::size_t i = 0; i < joint_directions.size(); i++)
    {
      joint_directions[i] *= -1;
    }
  }

  for (uint8_t i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);

    puma_motor_driver::Driver driver(gateway_, joint_can_ids[i], joint_names[i]);
    driver.clearMsgCache();
    driver.setEncoderCPR(encoder_cpr_);
    driver.setGearRatio(gear_ratio_ * joint_directions[i]);
    driver.setMode(puma_motor_msgs::Status::MODE_SPEED, gain_p_, gain_i_, gain_d_);
    drivers_.push_back(driver);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  multi_driver_node_.reset(new puma_motor_driver::MultiDriverNode(nh_, drivers_));
}

void DingoHardware::init()
{
  while (!connectIfNotConnected())
  {
    ros::Duration(1.0).sleep();
  }
}

bool DingoHardware::connectIfNotConnected()
{
  if (!gateway_.isConnected())
  {
    if (!gateway_.connect())
    {
      ROS_ERROR("Error connecting to motor driver gateway. Retrying in 1 second.");
      return false;
    }
    else
    {
      ROS_INFO("Connection to motor driver gateway successful.");
    }
  }
  return true;
}

void DingoHardware::configure()
{
  for (auto& driver : drivers_)
  {
    driver.configureParams();
  }
}

void DingoHardware::verify()
{
  for (auto& driver : drivers_)
  {
    driver.verifyParams();
  }
}

bool DingoHardware::areAllDriversActive()
{
  for (auto& driver : drivers_)
  {
    if (!driver.isConfigured())
    {
      return false;
    }
  }
  return true;
}

bool DingoHardware::isActive()
{
  if (active_ == false && this->areAllDriversActive())
  {
    active_ = true;
    multi_driver_node_->activePublishers(active_);
    ROS_INFO("Dingo Hardware Active");
  }
  else if (!this->areAllDriversActive() && active_ == true)
  {
    active_ = false;
  }

  if (!active_)
  {
    ROS_WARN_THROTTLE(60, "Dingo Hardware Inactive");
  }

  return active_;
}

void DingoHardware::powerHasNotReset()
{
  // Checks to see if power flag has been reset for each driver
  for (auto& driver : drivers_)
  {
    if (driver.lastPower() != 0)
    {
      active_ = false;
      ROS_WARN("There was a power reset on Dev: %d, will reconfigure all drivers.", driver.deviceNumber());
      multi_driver_node_->activePublishers(active_);
      for (auto& driver : drivers_)
      {
        driver.resetConfiguration();
      }
    }
  }
}

bool DingoHardware::inReset()
{
  return !active_;
}

void DingoHardware::requestData()
{
  for (auto& driver : drivers_)
  {
    driver.requestFeedbackPowerState();
  }
}

void DingoHardware::updateJointsFromHardware()
{
  uint8_t index = 0;
  for (auto& driver : drivers_)
  {
    Joint* f = &joints_[index];
    f->effort = driver.lastCurrent();
    f->position = driver.lastPosition();
    f->velocity = driver.lastSpeed();
    index++;
  }
}

void DingoHardware::command()
{
  uint8_t i = 0;
  for (auto& driver : drivers_)
  {
    driver.commandSpeed(joints_[i].velocity_command);
    i++;
  }
}

void DingoHardware::canRead()
{
  puma_motor_driver::Message recv_msg;
  while (gateway_.recv(&recv_msg))
  {
    for (auto& driver : drivers_)
    {
      driver.processMessage(recv_msg);
    }
  }
}

}  // namespace dingo_base
