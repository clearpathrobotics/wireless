
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"

#include <dynamic_reconfigure/server.h>
#include <wireless_quality/PublisherConfig.h>


sensor_msgs::CompressedImage msg;
ros::Publisher pub;
ros::Timer pub_timer;

void pub_callback(const ros::TimerEvent&) {
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
}

void config_callback(wireless_quality::PublisherConfig &config, uint32_t level)
{ 
    msg.data.resize(config.bytes_per_second / config.hz);
    pub_timer.setPeriod(ros::Duration(1.0 / config.hz));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "quality_publisher");

    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::CompressedImage>("data", 5);
    pub_timer = nh.createTimer(ros::Duration(1.0), pub_callback);

    dynamic_reconfigure::Server<wireless_quality::PublisherConfig> config_srv;
    dynamic_reconfigure::Server<wireless_quality::PublisherConfig>::CallbackType f;
    f = boost::bind(&config_callback, _1, _2);
    config_srv.setCallback(f);
    
    ros::spin();
}
