#include <rclcpp/rclcpp.hpp>
#include "pd_position_controller_simple.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("pid_position_controller_simple_node", node_options);

    PIDPositionController controller(nh);

    rclcpp::spin(nh);
    return 0;
}