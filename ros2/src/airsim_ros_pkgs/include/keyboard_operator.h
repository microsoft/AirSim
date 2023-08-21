//
// Created by larr-laptop on 23. 8. 19.
//

#ifndef AIRSIM_ROS_KEYBOARD_OPERATOR_H
#define AIRSIM_ROS_KEYBOARD_OPERATOR_H
#include "rclcpp/rclcpp.hpp"
#include "airsim_interfaces/srv/keyboard_input.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <csignal>
#include <termios.h>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <Eigen/Core>
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
using LocalGoalPose = geometry_msgs::msg::PoseStamped;
#define KEYBOARD_O 0x6f
#define KEYBOARD_P 0x70
#define KEYBOARD_K 0x6b
#define KEYBOARD_L 0x6c
#define KEYBOARD_W 0x77
#define KEYBOARD_S 0x73
#define KEYBOARD_A 0x61
#define KEYBOARD_D 0x64
#define KEYBOARD_Z 0x7A
#define KEYBOARD_C 0x63
#define KEYBOARD_Q 0x71
#define KEYBOARD_E 0x65


namespace keyboard_operator
{
using KeyboardInputService = airsim_interfaces::srv::KeyboardInput;
using KeyboardInputServer = rclcpp::Service<KeyboardInputService>::SharedPtr;

void quit(int sig);


class KeyboardOperator : public rclcpp::Node
{
public:
    KeyboardOperator();
    void KeyLoop();
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_goal_publisher_;
    rclcpp::TimerBase::SharedPtr keyboard_input_timer_;
    LocalGoalPose pose_des_keyboard_;

    void Publish();
    double increment_xyz_ = 0.02;
    double increment_yaw_ = 3.141592 / 12;

    bool move_mav(double dx, double dy, double dz, double dyaw);
};

}

#endif //AIRSIM_ROS_KEYBOARD_OPERATOR_H
