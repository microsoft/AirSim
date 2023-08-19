//
// Created by larr-laptop on 23. 8. 19.
//
#include "keyboard_operator.h"

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<keyboard_operator::KeyboardOperator>());
    rclcpp::shutdown();
    return 0;
}