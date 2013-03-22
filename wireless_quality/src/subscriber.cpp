
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Int32.h"
#include "wireless_msgs/Quality.h"


wireless_msgs::Quality quality_msg;

void cb(const sensor_msgs::CompressedImage& msg) {
    ROS_INFO("test!");
    ros::Duration latency = ros::Time::now() - msg.header.stamp;
    quality_msg.latency_measurements.push_back(latency.toSec());
}

void cb_test(const std_msgs::Int32& msg) {
    ROS_INFO("testy!");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "quality_subscriber");

    ros::NodeHandle nh;
    ROS_INFO("pop!");

    double hz;
    bool unreliable;

    // Publication of quality measures
    nh.param<bool>("unreliable", unreliable, true);
    nh.param<double>("hz", hz, 1.0);

    quality_msg.latency_measurements.reserve(30);
    ros::Publisher quality_pub = nh.advertise<wireless_msgs::Quality>("quality", 5);

    ros::Subscriber sub = nh.subscribe("data", 1, cb);
    ros::Subscriber sub2 = nh.subscribe("test", 1, cb_test);

    ros::Rate r(hz);
    while (ros::ok())
    {
        r.sleep();
        quality_msg.header.stamp = ros::Time::now();
        quality_pub.publish(quality_msg);
        quality_msg.latency_measurements.resize(0);
    }
}
