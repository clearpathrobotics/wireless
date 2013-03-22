
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "quality_publisher");

    ros::NodeHandle nh;

    int32_t hz, bytes_per_second, bytes_per_message;

    // Publication rate 
    nh.param<int32_t>("hz", hz, 10);
    nh.param<int32_t>("bytes_per_second", bytes_per_second, 1000000);
    bytes_per_message = bytes_per_second / hz;

    ros::Publisher pub = nh.advertise<sensor_msgs::CompressedImage>("data", 5);

    sensor_msgs::CompressedImage msg;
    msg.format = "dummy";
    //msg.data.resize(bytes_per_message);

    ros::Rate r(hz);
    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        r.sleep();
    }
}
