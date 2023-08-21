//
// Created by larr-laptop on 23. 8. 19.
//
#include "keyboard_operator.h"

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    keyboard_operator::KeyboardOperator keyboard_drone;
    signal(SIGINT,keyboard_operator::quit);
    keyboard_drone.KeyLoop();
    return 0;
}