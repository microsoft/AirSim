#include "pd_position_controller_simple.h"
using std::placeholders::_1;

bool PIDParams::load_from_rosparams(const rclcpp::Node& nh)
{
    bool found = true;

    found = found && nh.get_parameter("kp_x", kp_x);
    found = found && nh.get_parameter("kp_y", kp_y);
    found = found && nh.get_parameter("kp_z", kp_z);
    found = found && nh.get_parameter("kp_yaw", kp_yaw);

    found = found && nh.get_parameter("kd_x", kd_x);
    found = found && nh.get_parameter("kd_y", kd_y);
    found = found && nh.get_parameter("kd_z", kd_z);
    found = found && nh.get_parameter("kd_yaw", kd_yaw);

    found = found && nh.get_parameter("reached_thresh_xyz", reached_thresh_xyz);
    found = found && nh.get_parameter("reached_yaw_degrees", reached_yaw_degrees);

    return found;
}

bool DynamicConstraints::load_from_rosparams(const rclcpp::Node& nh)
{
    bool found = true;

    found = found && nh.get_parameter("max_vel_horz_abs", max_vel_horz_abs);
    found = found && nh.get_parameter("max_vel_vert_abs", max_vel_vert_abs);
    found = found && nh.get_parameter("max_yaw_rate_degree", max_yaw_rate_degree);

    return found;
}

PIDPositionController::PIDPositionController(const rclcpp::Node& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), has_odom_(false), has_goal_(false), reached_goal_(false), got_goal_once_(false), has_home_geo_(false), use_eth_lib_for_geodetic_conv_(true)
{
    params_.load_from_rosparams(nh_private_);
    constraints_.load_from_rosparams(nh_);
    initialize_ros();
    reset_errors();
}

void PIDPositionController::reset_errors()
{
    prev_error_.x = 0.0;
    prev_error_.y = 0.0;
    prev_error_.z = 0.0;
    prev_error_.yaw = 0.0;
}

void PIDPositionController::initialize_ros()
{
    vel_cmd_ = airsim_interfaces::msg::VelCmd();
    // ROS params
    double update_control_every_n_sec;
    nh_private_.getParam("update_control_every_n_sec", update_control_every_n_sec);

    std::string vehicle_name;

    while (vehicle_name == "") {
        nh_private_.getParam("/vehicle_name", vehicle_name);
        RCLCPP_INFO_STREAM(nh_.get_logger() ,"Waiting vehicle name");
    }

    // ROS publishers
    airsim_vel_cmd_world_frame_pub_ = nh_private_.advertise<airsim_interfaces::msg::VelCmd>("/airsim_node/" + vehicle_name + "/vel_cmd_world_frame", 1);

    // ROS subscribers
    airsim_odom_sub_ = nh_.create_subscription<nav_msgs::msg::Odometry>("/airsim_node/" + vehicle_name + "/odom_local_ned", 50, std::bind(&PIDPositionController::airsim_odom_cb, this, _1));
    home_geopoint_sub_ = nh_.create_subscription<airsim_interfaces::msg::GPSYaw>("/airsim_node/home_geo_point", 50, std::bind(&PIDPositionController::home_geopoint_cb, this, _1));
    // todo publish this under global nodehandle / "airsim node" and hide it from user
    local_position_goal_srvr_ = nh_.advertiseService("/airsim_node/local_position_goal", &PIDPositionController::local_position_goal_srv_cb, this);
    local_position_goal_override_srvr_ = nh_.advertiseService("/airsim_node/local_position_goal/override", &PIDPositionController::local_position_goal_srv_override_cb, this);
    gps_goal_srvr_ = nh_.advertiseService("/airsim_node/gps_goal", &PIDPositionController::gps_goal_srv_cb, this);
    gps_goal_override_srvr_ = nh_.advertiseService("/airsim_node/gps_goal/override", &PIDPositionController::gps_goal_srv_override_cb, this);

    // ROS timers
    update_control_cmd_timer_ = nh_private_.createTimer(ros::Duration(update_control_every_n_sec), &PIDPositionController::update_control_cmd_timer_cb, this);
}

void PIDPositionController::airsim_odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    has_odom_ = true;
    curr_odom_ = *odom_msg;
    curr_position_.x = odom_msg->pose.pose.position.x;
    curr_position_.y = odom_msg->pose.pose.position.y;
    curr_position_.z = odom_msg->pose.pose.position.z;
    curr_position_.yaw = utils::get_yaw_from_quat_msg(odom_msg->pose.pose.orientation);
}

// todo maintain internal representation as eigen vec?
// todo check if low velocity if within thresh?
// todo maintain separate errors for XY and Z
void PIDPositionController::check_reached_goal()
{
    double diff_xyz = sqrt((target_position_.x - curr_position_.x) * (target_position_.x - curr_position_.x) + (target_position_.y - curr_position_.y) * (target_position_.y - curr_position_.y) + (target_position_.z - curr_position_.z) * (target_position_.z - curr_position_.z));

    double diff_yaw = math_common::angular_dist(target_position_.yaw, curr_position_.yaw);

    // todo save this in degrees somewhere to avoid repeated conversion
    if (diff_xyz < params_.reached_thresh_xyz && diff_yaw < math_common::deg2rad(params_.reached_yaw_degrees))
        reached_goal_ = true;
}

bool PIDPositionController::local_position_goal_srv_cb(airsim_interfaces::srv::SetLocalPosition::Request& request, airsim_interfaces::srv::SetLocalPosition::Response& response)
{
    // this tells the update timer callback to not do active hovering
    if (!got_goal_once_)
        got_goal_once_ = true;

    if (has_goal_ && !reached_goal_) {
        // todo maintain array of position goals
        RCLCPP_ERROR_STREAM(nh_.get_logger(), "[PIDPositionController] denying position goal request. I am still following the previous goal");
        return false;
    }

    if (!has_goal_) {
        target_position_.x = request.x;
        target_position_.y = request.y;
        target_position_.z = request.z;
        target_position_.yaw = request.yaw;
        RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] got goal: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

        // todo error checks
        // todo fill response
        has_goal_ = true;
        reached_goal_ = false;
        reset_errors(); // todo
        return true;
    }

    // Already have goal, and have reached it
    RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] Already have goal and have reached it");
    return false;
}

bool PIDPositionController::local_position_goal_srv_override_cb(airsim_interfaces::srv::SetLocalPosition::Request& request, airsim_interfaces::srv::SetLocalPosition::Response& response)
{
    // this tells the update timer callback to not do active hovering
    if (!got_goal_once_)
        got_goal_once_ = true;

    target_position_.x = request.x;
    target_position_.y = request.y;
    target_position_.z = request.z;
    target_position_.yaw = request.yaw;
    RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] got goal: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

    // todo error checks
    // todo fill response
    has_goal_ = true;
    reached_goal_ = false;
    reset_errors(); // todo
    return true;
}

void PIDPositionController::home_geopoint_cb(const airsim_interfaces::msg::GPSYaw::SharedPtr gps_msg)
{
    if (has_home_geo_)
        return;
    gps_home_msg_ = *gps_msg;
    has_home_geo_ = true;
    RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] GPS reference initializing " << gps_msg->latitude << ", " << gps_msg->longitude << ", " << gps_msg->altitude);
    geodetic_converter_.initialiseReference(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
}

// todo do relative altitude, or add an option for the same?
bool PIDPositionController::gps_goal_srv_cb(airsim_interfaces::srv::SetGPSPosition::Request& request, airsim_interfaces::srv::SetGPSPosition::Response& response)
{
    if (!has_home_geo_) {
        RCLCPP_ERROR_STREAM(nh_.get_logger(), "[PIDPositionController] I don't have home GPS coord. Can't go to GPS goal!");
        response.success = false;
    }

    // convert GPS goal to NED goal

    if (!has_goal_) {
        msr::airlib::GeoPoint goal_gps_point(request.latitude, request.longitude, request.altitude);
        msr::airlib::GeoPoint gps_home(gps_home_msg_.latitude, gps_home_msg_.longitude, gps_home_msg_.altitude);
        bool use_eth_lib = true;
        if (use_eth_lib_for_geodetic_conv_) {
            double initial_latitude, initial_longitude, initial_altitude;
            geodetic_converter_.getReference(&initial_latitude, &initial_longitude, &initial_altitude);
            double n, e, d;
            geodetic_converter_.geodetic2Ned(request.latitude, request.longitude, request.altitude, &n, &e, &d);
            // RCLCPP_INFO_STREAM("[PIDPositionController] geodetic_converter_ GPS reference initialized correctly (lat long in radians) " << initial_latitude << ", "<< initial_longitude << ", " << initial_altitude);
            target_position_.x = n;
            target_position_.y = e;
            target_position_.z = d;
        }
        else // use airlib::GeodeticToNedFast
        {
            RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] home geopoint: lat=" << gps_home.latitude << " long=" << gps_home.longitude << " alt=" << gps_home.altitude << " yaw="
                                                                          << "todo");
            msr::airlib::Vector3r ned_goal = msr::airlib::EarthUtils::GeodeticToNedFast(goal_gps_point, gps_home);
            target_position_.x = ned_goal[0];
            target_position_.y = ned_goal[1];
            target_position_.z = ned_goal[2];
        }

        target_position_.yaw = request.yaw; // todo
        RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] got GPS goal: lat=" << goal_gps_point.latitude << " long=" << goal_gps_point.longitude << " alt=" << goal_gps_point.altitude << " yaw=" << target_position_.yaw);
        RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] converted NED goal is: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

        // todo error checks
        // todo fill response
        has_goal_ = true;
        reached_goal_ = false;
        reset_errors(); // todo
        return true;
    }

    // Already have goal, this shouldn't happen
    RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] Goal already received, ignoring!");
    return false;
}

// todo do relative altitude, or add an option for the same?
bool PIDPositionController::gps_goal_srv_override_cb(airsim_interfaces::srv::SetGPSPosition::Request& request, airsim_interfaces::srv::SetGPSPosition::Response& response)
{
    if (!has_home_geo_) {
        RCLCPP_ERROR_STREAM(nh_.get_logger(), "[PIDPositionController] I don't have home GPS coord. Can't go to GPS goal!");
        response.success = false;
    }

    // convert GPS goal to NED goal

    msr::airlib::GeoPoint goal_gps_point(request.latitude, request.longitude, request.altitude);
    msr::airlib::GeoPoint gps_home(gps_home_msg_.latitude, gps_home_msg_.longitude, gps_home_msg_.altitude);
    bool use_eth_lib = true;
    if (use_eth_lib_for_geodetic_conv_) {
        double initial_latitude, initial_longitude, initial_altitude;
        geodetic_converter_.getReference(&initial_latitude, &initial_longitude, &initial_altitude);
        double n, e, d;
        geodetic_converter_.geodetic2Ned(request.latitude, request.longitude, request.altitude, &n, &e, &d);
        // RCLCPP_INFO_STREAM("[PIDPositionController] geodetic_converter_ GPS reference initialized correctly (lat long in radians) " << initial_latitude << ", "<< initial_longitude << ", " << initial_altitude);
        target_position_.x = n;
        target_position_.y = e;
        target_position_.z = d;
    }
    else // use airlib::GeodeticToNedFast
    {
        RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] home geopoint: lat=" << gps_home.latitude << " long=" << gps_home.longitude << " alt=" << gps_home.altitude << " yaw="
                                                                      << "todo");
        msr::airlib::Vector3r ned_goal = msr::airlib::EarthUtils::GeodeticToNedFast(goal_gps_point, gps_home);
        target_position_.x = ned_goal[0];
        target_position_.y = ned_goal[1];
        target_position_.z = ned_goal[2];
    }

    target_position_.yaw = request.yaw; // todo
    RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] got GPS goal: lat=" << goal_gps_point.latitude << " long=" << goal_gps_point.longitude << " alt=" << goal_gps_point.altitude << " yaw=" << target_position_.yaw);
    RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] converted NED goal is: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

    // todo error checks
    // todo fill response
    has_goal_ = true;
    reached_goal_ = false;
    reset_errors(); // todo
    return true;
}

void PIDPositionController::update_control_cmd_timer_cb(const ros::TimerEvent& event)
{
    // todo check if odometry is too old!!
    // if no odom, don't do anything.
    if (!has_odom_) {
        RCLCPP_ERROR_STREAM(nh_.get_logger(), "[PIDPositionController] Waiting for odometry!");
        return;
    }

    if (has_goal_) {
        check_reached_goal();
        if (reached_goal_) {
            RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] Reached goal! Hovering at position.");
            has_goal_ = false;
            // dear future self, this function doesn't return coz we need to keep on actively hovering at last goal pose. don't act smart
        }
        else {
            RCLCPP_INFO_STREAM(nh_.get_logger(), "[PIDPositionController] Moving to goal.");
        }
    }

    // only compute and send control commands for hovering / moving to pose, if we received a goal at least once in the past
    if (got_goal_once_) {
        compute_control_cmd();
        enforce_dynamic_constraints();
        publish_control_cmd();
    }
}

void PIDPositionController::compute_control_cmd()
{
    curr_error_.x = target_position_.x - curr_position_.x;
    curr_error_.y = target_position_.y - curr_position_.y;
    curr_error_.z = target_position_.z - curr_position_.z;
    curr_error_.yaw = math_common::angular_dist(curr_position_.yaw, target_position_.yaw);

    double p_term_x = params_.kp_x * curr_error_.x;
    double p_term_y = params_.kp_y * curr_error_.y;
    double p_term_z = params_.kp_z * curr_error_.z;
    double p_term_yaw = params_.kp_yaw * curr_error_.yaw;

    double d_term_x = params_.kd_x * prev_error_.x;
    double d_term_y = params_.kd_y * prev_error_.y;
    double d_term_z = params_.kd_z * prev_error_.z;
    double d_term_yaw = params_.kp_yaw * prev_error_.yaw;

    prev_error_ = curr_error_;

    vel_cmd_.twist.linear.x = p_term_x + d_term_x;
    vel_cmd_.twist.linear.y = p_term_y + d_term_y;
    vel_cmd_.twist.linear.z = p_term_z + d_term_z;
    vel_cmd_.twist.angular.z = p_term_yaw + d_term_yaw; // todo
}

void PIDPositionController::enforce_dynamic_constraints()
{
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
    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_yaw_rate_degree) {
        // todo just add a sgn funciton in common utils? return double to be safe.
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_yaw_rate_degree;
    }
}

void PIDPositionController::publish_control_cmd()
{
    airsim_vel_cmd_world_frame_pub_.publish(vel_cmd_);
}