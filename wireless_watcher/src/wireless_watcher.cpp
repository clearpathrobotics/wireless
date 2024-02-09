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

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <chrono>
#include <thread>
#include <unordered_map>
#include <memory>
#include <cstdio>
#include <dirent.h>

#include "wireless_watcher/wireless_watcher.hpp"

using namespace std::chrono_literals;


WirelessWatcher::WirelessWatcher() : rclcpp::Node("wireless_watcher") {
    this->declare_parameter("hz", 1.0);
    this->declare_parameter("dev", "");
    this->declare_parameter("connected_topic", "connected");
    this->declare_parameter("connection_topic", "connection");

    hz = this->get_parameter("hz").as_double();
    dev = this->get_parameter("dev").as_string();
    connected_topic = this->get_parameter("connected_topic").as_string();
    connection_topic = this->get_parameter("connection_topic").as_string();

    if (dev.empty()) {
        std::vector<std::string> wldevs;
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir(SYS_NET_PATH)) != NULL) {
            while ((ent = readdir(dir)) != NULL) {
                std::string dev_name = ent->d_name;
                if (dev_name.compare(0, 2, "wl") == 0 || dev_name.compare(0, 4, "wifi") == 0) {
                    wldevs.push_back(dev_name);
                }
            }
            closedir(dir);
        } else {
            RCLCPP_FATAL(this->get_logger(), "Failed to open directory: %s", SYS_NET_PATH);
            return;
        }
        if (!wldevs.empty()) {
            dev = wldevs[0];
        } else {
            RCLCPP_FATAL(this->get_logger(), "No wireless device found to monitor.");
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Monitoring %s", dev.c_str());

    connected_pub_ = this->create_publisher<std_msgs::msg::Bool>(connected_topic, rclcpp::SensorDataQoS());
    connection_pub_ = this->create_publisher<wireless_msgs::msg::Connection>(connection_topic, rclcpp::SensorDataQoS());

    while (rclcpp::ok()) {
        try {
            std::string operstate_filepath = std::string(SYS_NET_PATH);
            operstate_filepath += "/";
            operstate_filepath += dev;
            operstate_filepath += "/operstate";
            std::ifstream operstate_file(operstate_filepath.c_str());
            std::string operstate;
            operstate_file >> operstate;
            connected_msg.data = operstate == "up";
        } catch (const std::exception& e) {
            connected_msg.data = false;
        }
        connected_pub_->publish(connected_msg);

        std::string iwconfig_output = exec_cmd("iwconfig " + dev);
        std::vector<std::string> fields_str = split(iwconfig_output, "\\s\\s+");
        
        std::unordered_map<std::string, std::string> fields_dict;
        fields_dict["dev"] = fields_str[0];
        fields_dict["type"] = fields_str[1];
        fields_dict["Access Point"] = split(fields_str[5], " ").back();

        for (size_t i = 2; i < fields_str.size(); ++i) {
            std::vector<std::string> field = split(fields_str[i], "[:=]");
            if (field.size() == 2) {
                fields_dict[field[0]] = field[1];
            }
        }

        if (fields_dict["Access Point"].find("Not-Associated") == std::string::npos) {
          connection_msg.bitrate = std::stof(split(fields_dict["Bit Rate"], " ")[0]);
          connection_msg.txpower = std::stoi(split(fields_dict["Tx-Power"], " ")[0]);
          connection_msg.signal_level = std::stoi(split(fields_dict["Signal level"], " ")[0]);

          // Strip quotations from ESSID
          std::string essid = fields_dict["ESSID"];
          essid.erase(std::remove(essid.begin(), essid.end(), '\"'), essid.end());
          connection_msg.essid = essid;
      
          connection_msg.bssid = fields_dict["Access Point"];
          connection_msg.frequency = std::stof(split(fields_dict["Frequency"], " ")[0]);

          // Calculate link_quality from Link Quality
          std::string link_quality_str = fields_dict["Link Quality"];
          connection_msg.link_quality_raw = link_quality_str;
          size_t delimiter_pos = link_quality_str.find("/");
          if (delimiter_pos != std::string::npos) {
              int num = std::stoi(link_quality_str.substr(0, delimiter_pos));
              int den = std::stoi(link_quality_str.substr(delimiter_pos + 1));
              connection_msg.link_quality = static_cast<float>(num) / den;
          }

          connection_pub_->publish(connection_msg);
        }
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0 / hz)));
    }
}

std::string WirelessWatcher::exec_cmd(const std::string& cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

std::vector<std::string> WirelessWatcher::split(const std::string& s, const std::string& delimiter) {
    std::regex regex(delimiter);
    std::sregex_token_iterator it(s.begin(), s.end(), regex, -1);
    std::vector<std::string> tokens{it, {}};
    return tokens;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WirelessWatcher>());
    rclcpp::shutdown();
    return 0;
}
