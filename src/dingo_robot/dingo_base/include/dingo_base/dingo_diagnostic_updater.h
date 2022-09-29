/**
 *  \file       dingo_diagnostic_updater.h
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

#ifndef DINGO_BASE_DINGO_DIAGNOSTIC_UPDATER_H
#define DINGO_BASE_DINGO_DIAGNOSTIC_UPDATER_H

#include <string>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "dingo_msgs/Status.h"

namespace dingo_base
{

class DingoDiagnosticUpdater : private diagnostic_updater::Updater
{
public:
  /** Sets up all the diagnostic publishers and callbacks */
  DingoDiagnosticUpdater();

  /** Reports general diagnostics (MCU uptime, stop status, etc)
   *  @param[out] stat Output into which status is written
   */
  void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /** Reports battery diagnostics (comparison to expected thresholds)
   *  @param[out] stat Output into which status is written
   */
  void batteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /** Reports voltage diagnostics (comparison to expected thresholds)
   *  @param[out] stat Output into which status is written
   */
  void voltageDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /** Reports current diagnostics (comparison to expected thresholds)
   *  @param[out] stat Output into which status is written
   */
  void currentDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /** Reports power diagnostics (comparison to expected thresholds)
   *  @param[out] stat Output into which status is written
   */
  void powerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /** Reports temperature diagnostics (comparison to expected thresholds)
   *  @param[out] stat Output into which status is written
   */
  void temperatureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  /** Callback that is run when a MCU status message is received
   *  @param[in] status Contents of MCU status message
   */
  void statusCallback(const dingo_msgs::Status::ConstPtr& status);

  /** Callback that is run when a battery state message is received
   *  @param[in] battery_state Contents of battery state message
   */
  void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& battery_state);

  /** Callback that is run when an IMU message is received
   *  @param[in] msg Contents of IMU message
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  /** Periodic callback used to see if wireless interface is connected
   *  @param[in] te Contents of the timer event
   */
  void wirelessMonitorCallback(const ros::TimerEvent& te);

private:
  /** ROS communication handle */
  ros::NodeHandle nh_;

  /** Used to subscribe for MCU status messages */
  ros::Subscriber status_sub_;

  /** Used to subscribe for battery state messages */
  ros::Subscriber battery_state_sub_;

  /** Last MCU status message */
  dingo_msgs::Status::ConstPtr last_status_;

  /** Last battery state message */
  sensor_msgs::BatteryState last_battery_state_;

  /** Frequency, in Hz, at which IMU updates are expected */
  double expected_imu_frequency_;

  /** Used to gather IMU diagnostic info */
  diagnostic_updater::TopicDiagnostic* imu_diagnostic_;

  /** Used to receive IMU messages */
  ros::Subscriber imu_sub_;

  /** The hostname of the machine on this code is running */
  char hostname_[1024];

  /** The name of the wireless interface (eg. wlan0) */
  std::string wireless_interface_;

  /** Used for wireless callback timer */
  ros::Timer wireless_monitor_timer_;

  /** Used to publish wifi connectivity updates */
  ros::Publisher wifi_connected_pub_;
};

}  // namespace dingo_base

#endif  // DINGO_BASE_DINGO_DIAGNOSTIC_UPDATER_H
