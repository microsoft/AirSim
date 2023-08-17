//
// Created by larr-planning on 23. 8. 17.
//
#include "pd_position_controller_simple_topic.h"

topic_mode::PDPositionController::PDPositionController(const rclcpp::NodeOptions &options_input) : Node(
        "pd_position_controller", options_input) {
    // Read Parameters
    get_parameter<double>("kp_x", params_.kp_x);
    get_parameter<double>("kp_y", params_.kp_y);
    get_parameter<double>("kp_z", params_.kp_z);
    get_parameter<double>("kp_yaw", params_.kp_yaw);
    get_parameter<double>("kd_x", params_.kd_x);
    get_parameter<double>("kd_y", params_.kd_y);
    get_parameter<double>("kd_z", params_.kd_z);
    get_parameter<double>("kd_yaw", params_.kd_yaw);
    get_parameter<double>("command_smoothing_weight", params_.command_smoothing_weight_);
    get_parameter<double>("max_vel_horz_abs", constraints_.max_vel_horz_abs);
    get_parameter<double>("max_vel_vert_abs", constraints_.max_vel_vert_abs);
    get_parameter<double>("max_yaw_rate_degree", constraints_.max_yaw_rate_degree);
    get_parameter<std::string>("vehicle_name", vehicle_name_);
    get_parameter<double>("control_update_rate", control_update_rate_);
    double control_period = 1.0/control_update_rate_;
    control_timer_ = this->create_wall_timer(std::chrono::duration<double>(control_period),
                                             std::bind(&PDPositionController::ControlTimerCallback, this));

    // Subscriber
    rclcpp::SubscriptionOptions options;
    {
        options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("~/drone_odometry", rclcpp::QoS(1), std::bind(
                &PDPositionController::AirsimOdomCallback, this, std::placeholders::_1), options);

        options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        airsim_local_goal_sub_ = create_subscription<nav_msgs::msg::Odometry>("~/local_goal_pose", rclcpp::QoS(1),
                                                                              std::bind(
                                                                                      &PDPositionController::AirsimLocalGoalCallback,
                                                                                      this, std::placeholders::_1),
                                                                              options);
    }
    // Publisher
    airsim_vel_cmd_world_frame_pub_ = create_publisher<airsim_interfaces::msg::VelCmd>(
            "/airsim_node/" + vehicle_name_ + "/vel_cmd_world_frame", rclcpp::QoS(1));
}

void topic_mode::PDPositionController::ControlTimerCallback() {
    if(has_local_goal_){
        ComputeControlCommand();
        EnforceDynamicConstraints();
        PublishCommand();
    }else{
        RCLCPP_INFO(get_logger(),"[PD Position Controller]: Local Goal Not Recieved");
    }
}

void topic_mode::PDPositionController::ComputeControlCommand() {
    current_error_.x = current_goal_pose_.x - current_pose_.x;
    current_error_.y = current_goal_pose_.y - current_pose_.y;
    current_error_.z = current_goal_pose_.z - current_pose_.z;
    current_error_.yaw = math_common::angular_dist(current_pose_.yaw, current_pose_.yaw);

    double p_term_x = params_.kp_x * current_error_.x;
    double p_term_y = params_.kp_y * current_error_.y;
    double p_term_z = params_.kp_z * current_error_.z;
    double p_term_yaw = params_.kp_yaw * current_error_.yaw;

    double d_term_x = params_.kd_x * previous_error_.x;
    double d_term_y = params_.kd_y * previous_error_.y;
    double d_term_z = params_.kd_z * previous_error_.z;
    double d_term_yaw = params_.kp_yaw * previous_error_.yaw;

    previous_error_ = current_error_;

    vel_cmd_.twist.linear.x = (1-params_.command_smoothing_weight_)*(p_term_x + d_term_x) + params_.command_smoothing_weight_*(vel_cmd_.twist.linear.x);
    vel_cmd_.twist.linear.y = (1-params_.command_smoothing_weight_)*(p_term_y + d_term_y) + params_.command_smoothing_weight_*(vel_cmd_.twist.linear.y);
    vel_cmd_.twist.linear.z = (1-params_.command_smoothing_weight_)*(p_term_z + d_term_z) + params_.command_smoothing_weight_*(vel_cmd_.twist.linear.z);
    vel_cmd_.twist.angular.z = p_term_yaw + d_term_yaw;

}

void topic_mode::PDPositionController::EnforceDynamicConstraints() {
    double vel_norm_horz = sqrt((vel_cmd_.twist.linear.x * vel_cmd_.twist.linear.x) + (vel_cmd_.twist.linear.y * vel_cmd_.twist.linear.y));

    if (vel_norm_horz > constraints_.max_vel_horz_abs) {
        vel_cmd_.twist.linear.x = (vel_cmd_.twist.linear.x / vel_norm_horz) * constraints_.max_vel_horz_abs;
        vel_cmd_.twist.linear.y = (vel_cmd_.twist.linear.y / vel_norm_horz) * constraints_.max_vel_horz_abs;
    }

    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_vel_vert_abs) {
        // todo just add a sgn funciton in common utils? return double to be safe.
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_vel_vert_abs;
    }
    // todo yaw limits
//    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_yaw_rate_degree) {
//        // todo just add a sgn funciton in common utils? return double to be safe.
//        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
//        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_yaw_rate_degree;
//    }

}

void topic_mode::PDPositionController::PublishCommand() {
    airsim_vel_cmd_world_frame_pub_->publish(vel_cmd_);
}

void topic_mode::PDPositionController::AirsimOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    has_local_odom_ = true;
    current_pose_.x = odom_msg->pose.pose.position.x;
    current_pose_.y = odom_msg->pose.pose.position.y;
    current_pose_.z = odom_msg->pose.pose.position.z;
    current_pose_.yaw = utils::get_yaw_from_quat_msg(odom_msg->pose.pose.orientation);
}

void topic_mode::PDPositionController::AirsimLocalGoalCallback(const nav_msgs::msg::Odometry::SharedPtr goal_msg) {
    has_local_goal_ = true;
    current_goal_pose_.x = goal_msg->pose.pose.position.x;
    current_goal_pose_.y = goal_msg->pose.pose.position.y;
    current_goal_pose_.z = goal_msg->pose.pose.position.z;
    current_goal_pose_.yaw = utils::get_yaw_from_quat_msg(goal_msg->pose.pose.orientation);
}



