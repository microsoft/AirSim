#include "rclcpp/rclcpp.hpp"
#include "pd_position_controller_simple.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("pid_position_controller_simple_node","pid_position_controller_simple_node");
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