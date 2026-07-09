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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
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

#include "wireless_watcher/wireless_watcher.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <ifaddrs.h>
#include <iostream>
#include <limits>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"

using namespace std::chrono_literals;

WirelessWatcher::WirelessWatcher()
    : rclcpp::Node("wireless_watcher"), updater_(this) {
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
        if (dev_name.compare(0, 2, "wl") == 0 ||
            dev_name.compare(0, 4, "wifi") == 0) {
          wldevs.push_back(dev_name);
        }
      }
      closedir(dir);
    } else {
      RCLCPP_FATAL(this->get_logger(), "Failed to open directory: %s",
                   SYS_NET_PATH);
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

  connected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      connected_topic, rclcpp::SensorDataQoS());
  connection_pub_ = this->create_publisher<wireless_msgs::msg::Connection>(
      connection_topic, rclcpp::SensorDataQoS());

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
      std::bind(&WirelessWatcher::timer_callback, this));

  updater_.setHardwareID(dev);
  updater_.add("Wi-Fi Monitor", this, &WirelessWatcher::diagnostic);
}

void WirelessWatcher::timer_callback() {
  try {
    std::string operstate_filepath = std::string(SYS_NET_PATH);
    operstate_filepath += "/";
    operstate_filepath += dev;
    operstate_filepath += "/operstate";
    std::ifstream operstate_file(operstate_filepath.c_str());
    std::string operstate;
    operstate_file >> operstate;
    connected_msg_.data = operstate == "up";
  } catch (const std::exception &e) {
    connected_msg_.data = false;
  }
  connected_pub_->publish(connected_msg_);

  if (!connected_msg_.data) {
    return;
  }

  std::string iw_link = exec_cmd("iw dev " + dev + " link 2>/dev/null");

  if (iw_link.find("Not connected") != std::string::npos || iw_link.empty()) {
    return;
  }

  std::smatch m;

  // BSSID: "Connected to xx:xx:xx:xx:xx:xx (on wlan0)"
  if (std::regex_search(iw_link, m,
                        std::regex(R"(Connected to ([\da-f:]+))"))) {
    connection_msg_.bssid = m[1].str();
  }

  // SSID
  if (std::regex_search(iw_link, m, std::regex(R"(\bSSID: (.+))"))) {
    std::string s = m[1].str();
    s.erase(s.find_last_not_of(" \t\r\n") + 1);
    connection_msg_.essid = s;
  }

  // freq in MHz, convert to GHz
  if (std::regex_search(iw_link, m, std::regex(R"(\bfreq: (\d+))"))) {
    connection_msg_.frequency = std::stod(m[1].str()) / 1000.0;
  }

  // signal in dBm
  if (std::regex_search(iw_link, m, std::regex(R"(\bsignal: (-?\d+) dBm)"))) {
    connection_msg_.signal_level = static_cast<int16_t>(std::stoi(m[1].str()));
  }

  // tx bitrate in MBit/s
  if (std::regex_search(iw_link, m,
                        std::regex(R"(\btx bitrate: ([\d.]+) MBit/s)"))) {
    try {
      connection_msg_.bitrate = std::stof(m[1].str());
    } catch (const std::invalid_argument &) {
      connection_msg_.bitrate = std::numeric_limits<float>::quiet_NaN();
    }
  } else {
    connection_msg_.bitrate = std::numeric_limits<float>::quiet_NaN();
  }

  // txpower from iw dev info
  std::string iw_info = exec_cmd("iw dev " + dev + " info 2>/dev/null");
  if (std::regex_search(iw_info, m, std::regex(R"(\btxpower ([\d.]+) dBm)"))) {
    connection_msg_.txpower = static_cast<int16_t>(std::stof(m[1].str()));
  }

  // Derive link quality from signal level (-100 dBm = 0%, -50 dBm = 100%)
  int quality_pct;
  if (connection_msg_.signal_level >= -50) {
    quality_pct = 100;
  } else if (connection_msg_.signal_level <= -100) {
    quality_pct = 0;
  } else {
    quality_pct = 2 * (connection_msg_.signal_level + 100);
  }
  connection_msg_.link_quality_raw = std::to_string(quality_pct) + "/100";
  connection_msg_.link_quality = static_cast<float>(quality_pct) / 100.0f;

  connection_pub_->publish(connection_msg_);
}

std::string WirelessWatcher::exec_cmd(const std::string &cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

std::vector<std::string> WirelessWatcher::split(const std::string &s,
                                                const std::string &delimiter) {
  std::regex regex(delimiter);
  std::sregex_token_iterator it(s.begin(), s.end(), regex, -1);
  std::vector<std::string> tokens{it, {}};
  return tokens;
}

void WirelessWatcher::diagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.add("Wireless Network Interface", dev);
  stat.add("Wi-Fi Connected", connected_msg_.data ? "True" : "False");

  if (!connected_msg_.data) {
    stat.summaryf(diagnostic_updater::DiagnosticStatusWrapper::WARN,
                  "%s Disconnected", dev.c_str());
    return;
  }
  stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "OK");

  ip_address_diag(dev, stat);
  stat.add("Frequency (GHz)", connection_msg_.frequency);
  stat.add("ESSID", connection_msg_.essid);
  stat.add("BSSID", connection_msg_.bssid);
  stat.add("Transmit Power (dBm)", connection_msg_.txpower);
  stat.add("Theoretical Max Bitrate (Mbps)", connection_msg_.bitrate);
  stat.add("Link Quality Raw", connection_msg_.link_quality_raw);
  stat.addf("Link Quality (%)", "%.1f", connection_msg_.link_quality * 100);
  stat.add("Signal Strength (dBm)", connection_msg_.signal_level);

  if (connection_msg_.signal_level < SIGNAL_STRENGTH_VERY_WEAK) {
    stat.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::WARN,
                       "Very Poor Signal Strength (%d dBm)",
                       connection_msg_.signal_level);
  } else if (connection_msg_.signal_level < SIGNAL_STRENGTH_WEAK) {
    stat.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::WARN,
                       "Poor Signal Strength (%d dBm)",
                       connection_msg_.signal_level);
  }
}

void WirelessWatcher::ip_address_diag(
    std::string dev, diagnostic_updater::DiagnosticStatusWrapper &stat) {

  struct ifaddrs *ptr_ifaddrs = nullptr, *entry;

  if (getifaddrs(&ptr_ifaddrs) == 0) {
    for (entry = ptr_ifaddrs; entry != nullptr; entry = entry->ifa_next) {
      // Find the requested interface and ensure it has an address
      if (std::string(entry->ifa_name) != dev || entry->ifa_addr == nullptr) {
        continue;
      }

      sa_family_t address_family = entry->ifa_addr->sa_family;
      // Skip if the address is not IPv4
      if (address_family != AF_INET) {
        continue;
      }
      char buffer[INET_ADDRSTRLEN] = {};
      inet_ntop(address_family,
                &((struct sockaddr_in *)(entry->ifa_addr))->sin_addr, buffer,
                INET_ADDRSTRLEN);

      stat.add("IP Address", std::string(buffer));

      if (entry->ifa_netmask != nullptr) {
        char buffer[INET_ADDRSTRLEN] = {
            0,
        };
        inet_ntop(address_family,
                  &((struct sockaddr_in *)(entry->ifa_netmask))->sin_addr,
                  buffer, INET_ADDRSTRLEN);

        stat.add("Netmask", std::string(buffer));
      } else {
        stat.add("Netmask", "Not found");
      }

      freeifaddrs(ptr_ifaddrs);
      return;
    }
  }
  stat.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::WARN,
                     "Failed to get IP addresses for %s", dev.c_str());
  stat.add("IP Address", "Not found");
  stat.add("Netmask", "Not found");

  if (ptr_ifaddrs != nullptr) {
    freeifaddrs(ptr_ifaddrs);
  }
  return;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WirelessWatcher>());
  rclcpp::shutdown();
  return 0;
}
