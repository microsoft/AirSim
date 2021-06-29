#include "ros/ros.h"
#include "airsim_ros_wrapper.h"
#include <ros/spinner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string host_ip = "localhost";
    nh_private.getParam("host_ip", host_ip);
    AirsimROSWrapper airsim_ros_wrapper(nh, nh_private, host_ip);

    if (airsim_ros_wrapper.is_used_img_timer_cb_queue_) {
        airsim_ros_wrapper.img_async_spinner_.start();
    }

    if (airsim_ros_wrapper.is_used_lidar_timer_cb_queue_) {
        airsim_ros_wrapper.lidar_async_spinner_.start();
    }

    ros::spin();

    return 0;
}