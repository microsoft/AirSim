#include "rclcpp/rclcpp.hpp"
#include "pd_position_controller_simple.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    const string node_name = "pid_position_controller_simple_node";
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared(node_name, node_name, node_options);
    std::shared_ptr<rclcpp::Node> nh_private = nh->create_sub_node("private");

    PIDPositionController controller(nh, nh /* nh_private */); //ToDo - do we really need nh_private?

    // int num_threads = 1;
    // ros::MultiThreadedSpinner multi_thread(num_threads);
    // multi_thread.spin();

    // ros::AsyncSpinner async_spinner(num_threads);
    // async_spinner.start();

    // single threaded spinner
    rclcpp::spin(nh);
    return 0;
}