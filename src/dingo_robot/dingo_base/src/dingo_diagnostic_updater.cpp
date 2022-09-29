/**
 *  \file       dingo_diagnostic_updater.cpp
 *  \brief      Diagnostic updating class for Dingo
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

#include <string>
#include <sys/types.h>
#include <ifaddrs.h>
#include <unistd.h>

#include "boost/algorithm/string/predicate.hpp"
#include "diagnostic_updater/update_functions.h"
#include "dingo_base/dingo_diagnostic_updater.h"
#include "dingo_base/dingo_power_levels.h"

namespace dingo_base
{

DingoDiagnosticUpdater::DingoDiagnosticUpdater()
{
  setHardwareID("unknown");
  gethostname(hostname_, 1024);

  add("General", this, &DingoDiagnosticUpdater::generalDiagnostics);
  add("Battery", this, &DingoDiagnosticUpdater::batteryDiagnostics);
  add("User voltage supplies", this, &DingoDiagnosticUpdater::voltageDiagnostics);
  add("Current consumption", this, &DingoDiagnosticUpdater::currentDiagnostics);
  add("Power consumption", this, &DingoDiagnosticUpdater::powerDiagnostics);
  add("Temperature", this, &DingoDiagnosticUpdater::temperatureDiagnostics);

  // The arrival of this message runs the update() method and triggers the above callbacks.
  status_sub_ = nh_.subscribe("mcu/status", 5, &DingoDiagnosticUpdater::statusCallback, this);

  // record the battery state
  battery_state_sub_ = nh_.subscribe("battery/status", 1, &DingoDiagnosticUpdater::batteryStateCallback, this);

  // These message frequencies are reported on separately.
  ros::param::param("~expected_imu_frequency", expected_imu_frequency_, 50.0);
  imu_diagnostic_ = new diagnostic_updater::TopicDiagnostic("/imu/data_raw", *this,
      diagnostic_updater::FrequencyStatusParam(&expected_imu_frequency_, &expected_imu_frequency_, 0.15),
      diagnostic_updater::TimeStampStatusParam(-1, 1.0));
  imu_sub_ = nh_.subscribe("/imu/data_raw", 5, &DingoDiagnosticUpdater::imuCallback, this);

  // Publish whether the wireless interface has an IP address every second.
  ros::param::param<std::string>("~wireless_interface", wireless_interface_, "wlan0");
  ROS_INFO_STREAM("Checking for wireless connectivity on interface: " << wireless_interface_);
  wifi_connected_pub_ = nh_.advertise<std_msgs::Bool>("wifi_connected", 1);
  wireless_monitor_timer_ =
    nh_.createTimer(ros::Duration(1.0), &DingoDiagnosticUpdater::wirelessMonitorCallback, this);
}

void DingoDiagnosticUpdater::generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.addf("MCU uptime", "%d seconds", last_status_->mcu_uptime.toSec());
  stat.add("External stop status", last_status_->external_stop_present ? "present" : "absent");
  stat.add("Run/stop status", last_status_->external_stop_present ? "running" : "stopped");

  if (!last_status_->drivers_active)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Stop loop open, platform immobilized.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System OK.");
  }
}

void DingoDiagnosticUpdater::batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Battery Voltage (V)", last_status_->measured_battery);

  // if the battery technology is unknown, assume the battery is SLA, as otherwise it should report this!
  if (last_battery_state_.power_supply_technology == sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
  {
    if (last_status_->measured_battery >= dingo_power::BATTERY_SLA_OVER_VOLT)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery overvoltage.");
    }
    else if (last_status_->measured_battery < dingo_power::VOLTAGE_ABSENT)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery voltage not detected, check BATT fuse.");
    }
    else if (last_status_->measured_battery <= dingo_power::BATTERY_SLA_CRITICAL_VOLT)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery critically under voltage.");
    }
    else if (last_status_->measured_battery <= dingo_power::BATTERY_SLA_LOW_VOLT)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery low voltage.");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK (SLA).");
    }
  }
  else
  {
    if (last_battery_state_.voltage >= dingo_power::BATTERY_LITHIUM_OVER_VOLT)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery overvoltage.");
    }
    else if (last_battery_state_.voltage < dingo_power::VOLTAGE_ABSENT)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery voltage not detected, check BATT fuse.");
    }
    else if (last_battery_state_.percentage <= dingo_power::BATTERY_LITHIUM_CRITICAL_PERCENT)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery charge critically low.");
    }
    else if (last_battery_state_.percentage <= dingo_power::BATTERY_LITHIUM_LOW_PERCENT)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery charge low.");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK (Li).");
    }
  }
}

void DingoDiagnosticUpdater::voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("12V Supply (V)", last_status_->measured_12v);
  stat.add("5V Supply (V)", last_status_->measured_5v);

  if (last_status_->measured_12v > dingo_power::USER_VOLT_12_HIGH ||
      last_status_->measured_5v > dingo_power::USER_VOLT_5_HIGH)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "User supply overvoltage. Accessories may be damaged.");
  }
  else if (last_status_->measured_12v < dingo_power::VOLTAGE_ABSENT ||
           last_status_->measured_5v < dingo_power::VOLTAGE_ABSENT)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "User supplies absent. Check tray fuses.");
  }
  else if (last_status_->measured_12v < dingo_power::USER_VOLT_12_LOW ||
           last_status_->measured_5v < dingo_power::USER_VOLT_5_LOW)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Voltage supplies undervoltage. Check loading levels.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "User supplies OK.");
  }
}

void DingoDiagnosticUpdater::currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Total current (A)", last_status_->total_current);

  if (last_status_->total_current > dingo_power::CURRENT_DRAW_CRITICAL)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Current draw critical.");
  }
  else if (last_status_->total_current > dingo_power::CURRENT_DRAW_WARNING)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Current draw warning.");
  }
  else if (last_status_->total_current > dingo_power::CURRENT_DRAW_REMINDER)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw requires monitoring.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Current draw nominal.");
  }
}

void DingoDiagnosticUpdater::powerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("Shore power connnected", last_status_->shore_power_connected);
  stat.add("Battery connnected", last_status_->battery_connected);
  stat.add("12V user power nominal", last_status_->power_12v_user_nominal);

  if (last_status_->shore_power_connected)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Shore power connected.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Running on battery.");
  }
}

void DingoDiagnosticUpdater::temperatureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("PCB temperature (C)", last_status_->pcb_temperature);
  stat.add("MCU temperature (C)", last_status_->mcu_temperature);

  if (last_status_->pcb_temperature > dingo_power::TEMPERATURE_CRITICAL)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "PCB temperature too HOT.");
  }
  else if (last_status_->pcb_temperature > dingo_power::TEMPERATURE_WARNING)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "PCB temperature getting warm.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "PCB temperature OK.");
  }

  if (last_status_->mcu_temperature > dingo_power::TEMPERATURE_CRITICAL)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "MCU temperature too HOT.");
  }
  else if (last_status_->mcu_temperature > dingo_power::TEMPERATURE_WARNING)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "MCU temperature getting warm.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "MCU temperature OK.");
  }
}

void DingoDiagnosticUpdater::statusCallback(const dingo_msgs::Status::ConstPtr& status)
{
  // Fresh data from the MCU, publish a diagnostic update.
  last_status_ = status;
  setHardwareID(hostname_ + '-' + last_status_->hardware_id);
  update();
}

void DingoDiagnosticUpdater::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& battery_state)
{
  last_battery_state_ = *battery_state;
}

void DingoDiagnosticUpdater::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_diagnostic_->tick(msg->header.stamp);
}

void DingoDiagnosticUpdater::wirelessMonitorCallback(const ros::TimerEvent& te)
{
  std_msgs::Bool wifi_connected_msg;
  wifi_connected_msg.data = false;

  // Get system structure of interface IP addresses.
  struct ifaddrs* ifa_head;
  if (getifaddrs(&ifa_head) != 0)
  {
    ROS_WARN("System call getifaddrs returned error code. Unable to detect network interfaces.");
    return;
  }

  // Iterate structure looking for the wireless interface.
  struct ifaddrs* ifa_current = ifa_head;
  while (ifa_current != NULL)
  {
    if (strcmp(ifa_current->ifa_name, wireless_interface_.c_str()) == 0)
    {
      int family = ifa_current->ifa_addr->sa_family;
      if (family == AF_INET || family == AF_INET6)
      {
        wifi_connected_msg.data = true;
        break;
      }
    }

    ifa_current = ifa_current->ifa_next;
  }

  // Free structure, publish result message.
  freeifaddrs(ifa_head);
  wifi_connected_pub_.publish(wifi_connected_msg);
}

}  // namespace dingo_base
