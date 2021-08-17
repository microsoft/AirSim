#include "rclcpp/rclcpp.hpp"
#include "airsim_ros_wrapper.h"
//#include <ros/spinner.h>

int main(int argc, char** argv)
{
    // ros::init(argc, argv, "airsim_node");
    // ros::NodeHandle nh;
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("airsim_node");
    std::shared_ptr<rclcpp::Node> nh_private = nh->create_sub_node("~");
    std::shared_ptr<rclcpp::Node> nh_img = nh->create_sub_node("img");
    std::shared_ptr<rclcpp::Node> nh_lidar = nh->create_sub_node("lidar");
    
   // ros::NodeHandle nh_private("~");

    std::string host_ip = "localhost";
    nh_private->get_parameter("host_ip", host_ip);
    AirsimROSWrapper airsim_ros_wrapper(nh, nh_private, nh_img, nh_lidar, host_ip);

    if (airsim_ros_wrapper.is_used_img_timer_cb_queue_) {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(nh_img);
        while (rclcpp::ok()) {
            executor.spin();
        }        
    //    airsim_ros_wrapper.img_async_spinner_.start();
    }

    if (airsim_ros_wrapper.is_used_lidar_timer_cb_queue_) {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(nh_lidar);
        while (rclcpp::ok()) {
            executor.spin();
        }     
     //   airsim_ros_wrapper.lidar_async_spinner_.start();
    }

    rclcpp::spin(nh);

    return 0;
}