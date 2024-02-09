#ifndef WIRELESS_WATCHER_HPP
#define WIRELESS_WATCHER_HPP

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "wireless_msgs/msg/connection.hpp"
#include "wireless_msgs/msg/network.hpp"

#define SYS_NET_PATH "/sys/class/net"

class WirelessWatcher : public rclcpp::Node {
public:
    WirelessWatcher();

private:
    double hz;
    std::string dev;
    std::string connected_topic;
    std::string connection_topic;


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
    rclcpp::Publisher<wireless_msgs::msg::Connection>::SharedPtr connection_pub_;
    std_msgs::msg::Bool connected_msg;
    wireless_msgs::msg::Connection connection_msg;

    std::string exec_cmd(const std::string& cmd);
    std::vector<std::string> split(const std::string& s, const std::string& delimiter);
};

#endif  // WIRELESS_WATCHER_HPP