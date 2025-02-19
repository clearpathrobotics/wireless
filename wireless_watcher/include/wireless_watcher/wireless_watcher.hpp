/**
 *
 *  \file
 *  \brief      Wireless Watcher Node
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
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
 *
 */

#ifndef WIRELESS_WATCHER_HPP
#define WIRELESS_WATCHER_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wireless_msgs/msg/connection.hpp"
#include "wireless_msgs/msg/network.hpp"

#define SYS_NET_PATH "/sys/class/net"
#define SIGNAL_STRENGTH_WEAK -67
#define SIGNAL_STRENGTH_VERY_WEAK -75

class WirelessWatcher : public rclcpp::Node {
public:
    WirelessWatcher();

private:
    // Parameters
    double hz;
    std::string dev;
    std::string connected_topic;
    std::string connection_topic;

    // Other Variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
    rclcpp::Publisher<wireless_msgs::msg::Connection>::SharedPtr connection_pub_;
    std_msgs::msg::Bool connected_msg_;
    wireless_msgs::msg::Connection connection_msg_;
    diagnostic_updater::Updater updater_;

    // Methods
    void timer_callback();
    std::string exec_cmd(const std::string& cmd);
    std::vector<std::string> split(const std::string& s, const std::string& delimiter);
    void diagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
};

#endif  // WIRELESS_WATCHER_HPP