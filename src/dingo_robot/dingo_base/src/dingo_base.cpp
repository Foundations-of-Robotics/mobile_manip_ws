/**
 *  \file       dingo_base.cpp
 *  \brief      Main entry point for dingo base
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

#include <chrono>  // NOLINT(build/c++11)
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>
#include <thread>  // NOLINT(build/c++11)

#include "controller_manager/controller_manager.h"
#include "dingo_base/dingo_diagnostic_updater.h"
#include "dingo_base/dingo_hardware.h"
#include "dingo_base/dingo_cooling.h"
#include "dingo_base/dingo_lighting.h"
#include "dingo_base/dingo_logger.h"
#include "puma_motor_driver/diagnostic_updater.h"
#include "ros/ros.h"
#include "rosserial_server/udp_socket_session.h"

using boost::asio::ip::udp;
using boost::asio::ip::address;

/** This is the main thread for monitoring and updating the robot.
 *  Assumes a separate thread is used for this function as it never returns.
 *  @param[in] rate Controls the rate at which each loop iteration is run
 *  @param[in] robot Handle to the hardware for the robot
 *  @param[in] cm Handle to the controller manager
 */
void control(ros::Rate rate, dingo_base::DingoHardware* robot, controller_manager::ControllerManager* cm)
{
  std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

  while (ros::ok())
  {
    // Calculate monotonic time elapsed
    std::chrono::steady_clock::time_point this_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    if (robot->isActive())
    {
      robot->powerHasNotReset();
      robot->updateJointsFromHardware();
    }
    else
    {
      robot->configure();
    }

    cm->update(ros::Time::now(), elapsed, robot->inReset());

    if (robot->isActive())
    {
      robot->command();
      robot->requestData();
    }
    else
    {
      robot->verify();
    }
    rate.sleep();
  }
}

/** Perform reads on CAN bus. Assumes a separate thread is used for this
 *  function as it never returns.
 *  @param[in] rate Controls the rate at which CAN bus is read
 *  @param[in] robot Handle to the hardware for the robot
 */
void canRead(ros::Rate rate, dingo_base::DingoHardware* robot)
{
  while (ros::ok())
  {
    robot->canRead();
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  const std::string node_name = "dingo_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh, pnh("~");

  // Create the socket rosserial server in a background ASIO event loop.
  boost::asio::io_service io_service;
  rosserial_server::UdpSocketSession* socket;
  boost::thread socket_thread;

  const std::string BASE_IP = "192.168.131.1";
  const std::string MCU_IP = "192.168.131.2";
  const uint32_t ROSSERIAL_PORT = 11411;
  const uint32_t LOGGER_PORT    = 11413;

  // Network MCU logger
  std::string logger_name = node_name + "_mcu";
  dingo_base::DingoLogger logger(logger_name, nh, 1000);
  logger.configure(BASE_IP, LOGGER_PORT, MCU_IP, LOGGER_PORT);
  logger.init();
  std::thread logger_thread = logger.runThread();

  socket = new rosserial_server::UdpSocketSession(io_service,
      udp::endpoint(address::from_string(BASE_IP), ROSSERIAL_PORT),
      udp::endpoint(address::from_string(MCU_IP), ROSSERIAL_PORT));
  socket_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  std::string canbus_dev;
  pnh.param<std::string>("canbus_dev", canbus_dev, "can0");
  puma_motor_driver::SocketCANGateway gateway(canbus_dev);

  bool dingo_omni = false;
  pnh.param<bool>("dingo_omni", dingo_omni, false);
  dingo_base::DingoHardware dingo(nh, pnh, gateway, dingo_omni);

  // Configure the CAN connection
  dingo.init();
  // Create a thread to start reading can messages.
  std::thread can_read_thread(&canRead, ros::Rate(200), &dingo);

  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&dingo, controller_nh);
  std::thread control_thread(&control, ros::Rate(25), &dingo, &cm);

  // Lighting control.
  dingo_base::DingoLighting* lighting;
  lighting = new dingo_base::DingoLighting(&nh);

  // Create diagnostic updater, to update itself on the ROS thread.
  dingo_base::DingoDiagnosticUpdater dingo_diagnostic_updater;
  puma_motor_driver::PumaMotorDriverDiagnosticUpdater puma_motor_driver_diagnostic_updater;

  // Cooling control for the fans.
  dingo_base::DingoCooling* cooling;
  cooling = new dingo_base::DingoCooling(&nh);

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics
  ros::spin();

  logger_thread.join();

  return 0;
}
