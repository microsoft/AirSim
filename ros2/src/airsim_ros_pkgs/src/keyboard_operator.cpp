//
// Created by larr-laptop on 23. 8. 19.
//
#include "keyboard_operator.h"

int kfd = 0;
struct termios cooked, raw;

void keyboard_operator::quit(int sig)
{
    using namespace keyboard_operator;
    tcsetattr(kfd, TCSANOW, &cooked);
    rclcpp::shutdown();
    exit(0);
}

keyboard_operator::KeyboardOperator::KeyboardOperator()
    : Node("keyboard_operator")
{
    pose_des_keyboard_.pose.position.x = 0.0;
    pose_des_keyboard_.pose.position.y = 0.0;
    pose_des_keyboard_.pose.position.z = 0.0;
    pose_des_keyboard_.pose.orientation.w = 1.0;
    pose_des_keyboard_.pose.orientation.x = 0.0;
    pose_des_keyboard_.pose.orientation.y = 0.0;
    pose_des_keyboard_.pose.orientation.z = 0.0;

    local_goal_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/los_keeper/local_goal_pose",rclcpp::QoS(1));

    //    KeyboardInputServer  = this->create_service<KeyboardInputService>("~/")
}

void keyboard_operator::KeyboardOperator::KeyLoop()
{
    char c;
    bool dirty = false;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle.");

    for (;;) {
        //get the next event from the keyboard
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }
        Eigen::Vector4d move_pose;
        move_pose.setZero();

        switch (c) {
        case KEYBOARD_O: {
            increment_xyz_ -= 0.02;
            RCLCPP_INFO(get_logger(), "Decreased linear step. current: %f", increment_xyz_);
            break;
        }
        case KEYBOARD_P: {
            increment_xyz_ += 0.02;
            RCLCPP_INFO(get_logger(), "Increased linear step. current: %f", increment_xyz_);
            break;
        }
        case KEYBOARD_K: {
            increment_yaw_ -= 3.141592 / 12;
            RCLCPP_INFO(get_logger(), "Decreased yaw step. current: %f", increment_yaw_);
            break;
        }
        case KEYBOARD_L: {
            increment_yaw_ += 3.141592 / 12;
            RCLCPP_INFO(get_logger(), "Increased yaw step. current: %f", increment_yaw_);
            break;
        }
        case KEYBOARD_W: {
            move_pose(0) = increment_xyz_;
            RCLCPP_INFO(get_logger(),"Go forward.");
            break;
        }
        case KEYBOARD_S: {
            move_pose(0) = -increment_xyz_;
            RCLCPP_INFO(get_logger(),"Go backward.");
            break;
        }
        case KEYBOARD_A: {
            move_pose(1) = -increment_xyz_;
            RCLCPP_INFO(get_logger(),"Go left.");
            break;
        }
        case KEYBOARD_D: {
            move_pose(1) = increment_xyz_;
            RCLCPP_INFO(get_logger(),"Go right.");
            break;
        }
        case KEYBOARD_Z: {
            move_pose(2) = increment_xyz_;
            RCLCPP_INFO(get_logger(),"Go downward.");
            break;
        }
        case KEYBOARD_C: {
            move_pose(2) = -increment_xyz_;
            RCLCPP_INFO(get_logger(),"Go upward.");
            break;
        }
        case KEYBOARD_Q: {
            move_pose(3) = -increment_xyz_;
            RCLCPP_INFO(get_logger(),"Turn clockwise.");
            break;
        }
        case KEYBOARD_E: {
            move_pose(3) = increment_xyz_;
            RCLCPP_INFO(get_logger(),"Turn counter-clockwise.");
            break;
        }
        }
        move_mav(move_pose(0),move_pose(1),move_pose(2),move_pose(3));
    }
}
void keyboard_operator::KeyboardOperator::Publish()
{
}
bool keyboard_operator::KeyboardOperator::move_mav(double dx, double dy, double dz, double dyaw)
{
    Eigen::Vector3d dpose(dx, dy, dz);
    tf2::Quaternion q_cur_des;
    q_cur_des.setX(pose_des_keyboard_.pose.orientation.x);
    q_cur_des.setY(pose_des_keyboard_.pose.orientation.y);
    q_cur_des.setZ(pose_des_keyboard_.pose.orientation.z);
    q_cur_des.setW(pose_des_keyboard_.pose.orientation.w);

    geometry_msgs::msg::TransformStamped transfrom_cur;
    transfrom_cur.transform.translation.x = 0.0;
    transfrom_cur.transform.translation.y = 0.0;
    transfrom_cur.transform.translation.z = 0.0;
    transfrom_cur.transform.rotation.x = q_cur_des.x();
    transfrom_cur.transform.rotation.y = q_cur_des.y();
    transfrom_cur.transform.rotation.z = q_cur_des.z();
    transfrom_cur.transform.rotation.w = q_cur_des.w();

    Eigen::Isometry3d Twb = tf2::transformToEigen(transfrom_cur);
    Eigen::Vector3d dpose_w = Twb * dpose;
    // Step 1: modify xyz
    pose_des_keyboard_.pose.position.x += dpose_w(0);
    pose_des_keyboard_.pose.position.y += dpose_w(1);
    pose_des_keyboard_.pose.position.z += dpose_w(2);
    // Step 2: modify yaw direcion
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_cur_des).getEulerYPR(yaw, pitch, roll);
    yaw += dyaw; // here, we adjust yaw
    tf2::Quaternion q_des = tf2::Quaternion();
    q_des.setRPY(roll, pitch, yaw);
    pose_des_keyboard_.pose.orientation.x = q_des.getX();
    pose_des_keyboard_.pose.orientation.y = q_des.getY();
    pose_des_keyboard_.pose.orientation.z = q_des.getZ();
    pose_des_keyboard_.pose.orientation.w = q_des.getW();

    nav_msgs::msg::Odometry local_goal_info;
    local_goal_info.header.frame_id= "Chaser";
    local_goal_info.child_frame_id="Chaser/odom_local_ned";
    local_goal_info.pose.pose.position.x = pose_des_keyboard_.pose.position.x;
    local_goal_info.pose.pose.position.y = pose_des_keyboard_.pose.position.y;
    local_goal_info.pose.pose.position.z = pose_des_keyboard_.pose.position.z;
    local_goal_info.pose.pose.orientation.x = pose_des_keyboard_.pose.orientation.x;
    local_goal_info.pose.pose.orientation.y = pose_des_keyboard_.pose.orientation.y;
    local_goal_info.pose.pose.orientation.z = pose_des_keyboard_.pose.orientation.z;
    local_goal_info.pose.pose.orientation.w = pose_des_keyboard_.pose.orientation.w;
    local_goal_publisher_->publish(local_goal_info);
    return true;
}
