//
// Created by larr-planning on 23. 8. 17.
//
#include "pd_position_controller_simple_topic.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node_options =rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto pd_position_controller = std::make_shared<topic_mode::PDPositionController>(node_options);
    rclcpp::spin(pd_position_controller);
//    rclcpp::executors::MultiThreadedExecutor executor;
//    executor.add_node(pd_position_controller);
//    executor.spin();
    rclcpp::shutdown();
    return 0;
}