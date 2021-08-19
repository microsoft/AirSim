#include "rclcpp/rclcpp.hpp"
#include "airsim_ros_wrapper.h"
//#include <ros/spinner.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    const string node_name = "airsim_node";
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared(node_name, node_name, node_options);
 //   std::shared_ptr<rclcpp::Node> nh_private = nh->create_sub_node("~/");
    std::shared_ptr<rclcpp::Node> nh_img = nh->create_sub_node("img");
    std::shared_ptr<rclcpp::Node> nh_lidar = nh->create_sub_node("lidar");
    std::string host_ip;
    /* nh_private */nh->get_parameter("host_ip", host_ip);
    AirsimROSWrapper airsim_ros_wrapper(nh, nh /* nh_private */, nh_img, nh_lidar, host_ip);//ToDo - do we really need nh_private?

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