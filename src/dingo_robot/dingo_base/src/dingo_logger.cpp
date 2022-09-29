/**
 *  \file       dingo_logger.cpp
 *  \brief      Network logger class for Dingo
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
#include <vector>

#include "dingo_base/dingo_logger.h"

namespace dingo_base
{

using boost::asio::ip::udp;
using boost::asio::ip::address;

DingoLogger::DingoLogger(const std::string name, ros::NodeHandle& nh, const double rate) :
  name_(name),
  nh_(nh),
  rate_(ros::Rate(rate)),
  io_service_(),
  socket_(io_service_),
  is_running_(false),
  local_ip_("127.0.0.1"),
  local_port_(11413),
  remote_ip_(local_ip_),
  remote_port_(11413),
  regexp_("^\\[(.+)\\] (.+):(\\d+) (.+)$")
{
  local_name_ = name_ + "_logger";
  log_pub_ = nh_.advertise<rosgraph_msgs::Log>("/rosout", 100);
}

DingoLogger::~DingoLogger()
{
  stop();
}

void DingoLogger::configure(const std::string& local_ip, const uint32_t local_port,
                       const std::string& remote_ip, const uint32_t remote_port)
{
  local_ip_ = local_ip;
  local_port_ = local_port;
  remote_ip_ = remote_ip;
  remote_port_ = remote_port;
}

void DingoLogger::connect()
{
  try
  {
    socket_.open(udp::v4());
    ROS_INFO_STREAM(local_name_.c_str() << ": UDP socket opened");
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM(local_name_.c_str() << ": Could not open UDP socket: " << e.what());
  }

  local_endpoint_ = boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string(local_ip_), local_port_);
  remote_endpoint_ = boost::asio::ip::udp::endpoint(
    boost::asio::ip::address::from_string(remote_ip_), remote_port_);
  ROS_INFO_STREAM(local_name_.c_str() << " : Trying to connect to " << local_ip_ << ":" << local_port_);

  try
  {
    socket_.bind(local_endpoint_);
    ROS_INFO_STREAM(local_name_.c_str() << ": Bound Socket");
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM(local_name_.c_str() << ": Could not bind UDP socket: " << e.what());
  }
}

void DingoLogger::init()
{
  connect();
  is_running_ = true;
  setupReceive();
  udp_process_thread_ = this->processingThread();
}

void DingoLogger::stop()
{
  is_running_ = false;
  io_service_.stop();
  udp_process_thread_.join();
  socket_.close();
}

void DingoLogger::process()
{
  while (ros::ok() && is_running_)
  {
    std::string log_msg;
    if (gotNewLog(log_msg))
    {
      std::smatch sm;
      if (std::regex_match(log_msg, sm, regexp_))
      {
        rosgraph_msgs::Log log_msg;
        log_msg.header.stamp = ros::Time::now();
        log_msg.level = LogLevelMap.at(sm.str(1));
        log_msg.file = sm.str(2);
        log_msg.line = std::stoi(sm.str(3));
        log_msg.name = name_;
        log_msg.msg = sm.str(4);
        ROS_LOG(LogLevelMap.at(sm.str(1)), ROSCONSOLE_DEFAULT_NAME, "%s: %s", name_.c_str(), sm.str(4).c_str());
        log_pub_.publish(log_msg);
      }
    }
    rate_.sleep();
  }
}

bool DingoLogger::isConnected() const
{
  return is_running_;
}

std::thread DingoLogger::processingThread()
{
  return std::thread([=] { process(); });  // NOLINT
}

std::thread DingoLogger::runThread()
{
  return std::thread([&] { io_service_.run(); });  // NOLINT
}

void DingoLogger::setupReceive()
{
  socket_.async_receive_from(boost::asio::buffer(receive_udp_buffer_),
      remote_endpoint_, boost::bind(&DingoLogger::onUdpReceive, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void DingoLogger::onUdpReceive(const boost::system::error_code& ec, std::size_t bytes_transferred)
{
  std::lock_guard<std::mutex> lock(receive_queue_mutex_);
  udp_receive_queue_.push(std::string(receive_udp_buffer_.begin(), receive_udp_buffer_.begin() + bytes_transferred));
  if (!ec || ec == boost::asio::error::message_size)
  {
    setupReceive();
  }
  else
  {
    ROS_ERROR_STREAM(local_name_.c_str() << ": Receive Error: "  << ec.message());
  }
}

bool DingoLogger::gotNewLog(std::string& msg)
{
  std::lock_guard<std::mutex> lock(receive_queue_mutex_);
  if (udp_receive_queue_.empty())
  {
    return false;
  }
  msg = udp_receive_queue_.front();
  udp_receive_queue_.pop();
  return true;
}

}  // namespace dingo_base
