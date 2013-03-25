
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Int32.h"
#include "wireless_msgs/Quality.h"


wireless_msgs::Quality quality_msg;
ros::Publisher quality_pub;

void data_receive(const sensor_msgs::CompressedImage& msg) {
    static int32_t last_seq = 0;

    ros::Duration latency = ros::Time::now() - msg.header.stamp;
    quality_msg.latency_measurements.push_back(latency.toSec());
    quality_msg.messages_received++;
    
    quality_msg.message_lengths.push_back(msg.data.size()); 

    quality_msg.messages_missed += (msg.header.seq - last_seq) - 1;
    last_seq = msg.header.seq;
}

void quality_publish(const ros::TimerEvent&) {
    quality_msg.header.stamp = ros::Time::now();

    uint32_t num_messages = quality_msg.messages_received;
    quality_msg.total_length = 0;

    if (num_messages > 0) {
        double latency_total = 0;
        for (size_t i = 0; i < num_messages; ++i)
        {
            latency_total += quality_msg.latency_measurements[i];
            quality_msg.total_length += quality_msg.message_lengths[i];
        }
        quality_msg.latency_avg = latency_total / num_messages;
    } else {
        quality_msg.latency_avg = 0;
    }

    quality_pub.publish(quality_msg);
    quality_msg.latency_measurements.resize(0);
    quality_msg.message_lengths.resize(0);
    quality_msg.messages_missed = 0;
    quality_msg.messages_received = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quality_subscriber");

    ros::NodeHandle nh;
    ROS_INFO("pop2!");

    double hz;
    bool unreliable;

    // Publication of quality measures
    nh.param<bool>("unreliable", unreliable, true);
    nh.param<double>("hz", hz, 1.0);

    quality_msg.latency_measurements.reserve(30);
    quality_pub = nh.advertise<wireless_msgs::Quality>("quality", 5);

    ros::Subscriber data_sub = nh.subscribe("data", 1, data_receive, ros::TransportHints().unreliable());
    ros::Timer quality_timer = nh.createTimer(ros::Duration(1.0 / hz), quality_publish);

    ros::spin();
}
