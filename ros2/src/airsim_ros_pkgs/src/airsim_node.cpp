#include <rclcpp/rclcpp.hpp>
#include "airsim_ros_wrapper.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("airsim_node", node_options);
    
    std::string host_ip;
    nh->get_parameter("host_ip", host_ip);
    AirsimROSWrapper airsim_ros_wrapper(nh, host_ip);

    // Use MultiThreadedExecutor to handle multiple callback groups in parallel
    // Each sensor type (IMU 200Hz, Image 30Hz, Lidar) has its own callback group
    // This allows true parallel execution of sensor callbacks
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);  // 4 threads
    executor.add_node(nh);
    
    RCLCPP_INFO(nh->get_logger(), "Starting MultiThreadedExecutor with 4 threads for parallel sensor callbacks");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
