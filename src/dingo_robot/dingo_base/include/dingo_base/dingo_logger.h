/**
 *  \file       dingo_logger.h
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

#ifndef DINGO_BASE_DINGO_LOGGER_H
#define DINGO_BASE_DINGO_LOGGER_H

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>
#include <mutex>  // NOLINT(build/c++11)
#include <regex>  // NOLINT(build/c++11)
#include <string>
#include <thread>  // NOLINT(build/c++11)
#include <queue>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosgraph_msgs/Log.h>

namespace dingo_base
{

using boost::asio::ip::udp;
using boost::asio::ip::address;

const std::map<std::string, ros::console::levels::Level> LogLevelMap =
{
  {"DEBUG", ros::console::levels::Level::Debug},   // 1
  {"INFO",  ros::console::levels::Level::Info},    // 2
  {"WARN",  ros::console::levels::Level::Warn},    // 3
  {"ERROR", ros::console::levels::Level::Error},   // 4
  {"FATAL", ros::console::levels::Level::Fatal}    // 5
};

class DingoLogger
{
  public:
    DingoLogger(const std::string name, ros::NodeHandle& nh, const double rate);
    ~DingoLogger();

    void configure(const std::string& local_ip, const uint32_t local_port,
                   const std::string& remote_ip, const uint32_t remote_port);
    void connect();
    bool isConnected() const;
    void connectIfNotConnected();
    void init();
    void stop();
    void process();
    void queueToSend(const std::vector<uint8_t>& msg);
    void send(const std::vector<uint8_t>& msg);
    std::thread processingThread();
    std::thread runThread();

  private:
    void setupReceive();
    void onUdpReceive(const boost::system::error_code& ec, std::size_t bytes_transferred);
    bool gotNewLog(std::string& msg);
    void setLogLevel(ros::console::levels::Level level);

    std::string local_ip_, remote_ip_;
    uint32_t local_port_, remote_port_;
    udp::endpoint local_endpoint_, remote_endpoint_;
    boost::asio::io_service io_service_;
    udp::socket socket_;
    boost::array<uint8_t, 1024> receive_udp_buffer_;
    std::queue<std::string> udp_receive_queue_;
    std::mutex receive_queue_mutex_;
    std::thread udp_process_thread_;
    std::atomic<bool> is_running_;

    ros::NodeHandle nh_;
    ros::Publisher log_pub_;
    ros::Rate rate_;
    const std::regex regexp_;
    bool debug_;
    std::string name_;
    std::string local_name_;
};

}  // namespace dingo_base

#endif  // DINGO_BASE_DINGO_LOGGER_H
