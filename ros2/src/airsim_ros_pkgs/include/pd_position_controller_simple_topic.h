//
// Created by larr-planning on 23. 8. 17.
//

#ifndef AIRSIM_ROS_PKGS_PD_POSITION_CONTROLLER_SIMPLE_TOPIC_H
#define AIRSIM_ROS_PKGS_PD_POSITION_CONTROLLER_SIMPLE_TOPIC_H

#include "common/common_utils/StrictMode.hpp"

STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK

#include "rpc/rpc_error.h"

STRICT_MODE_ON

#include "common/common_utils/FileSystem.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <math.h>
#include <airsim_interfaces/msg/vel_cmd.hpp>
#include <airsim_interfaces/srv/set_local_position.hpp>
#include <airsim_interfaces/srv/set_gps_position.hpp>
#include <airsim_interfaces/msg/gps_yaw.hpp>
#include <geodetic_conv.hpp>
#include <math_common.h>
#include <utils.h>

namespace topic_mode {
    using RosTimer = rclcpp::TimerBase::SharedPtr;
    class PDControllerParams {
    public:
        double kp_x;
        double kp_y;
        double kp_z;
        double kp_yaw;
        double kd_x;
        double kd_y;
        double kd_z;
        double kd_yaw;
        double command_smoothing_weight_; // [0,1] // 0: follows current goal, 1: follows previous goal
        double reached_thresh_xyz;
        double reached_yaw_degrees;

        PDControllerParams()
                : kp_x(0.5), kp_y(0.5), kp_z(0.5), kp_yaw(0.5), kd_x(0.1), kd_y(0.1), kd_z(0.1), kd_yaw(0.1),
                  reached_thresh_xyz(0.5), reached_yaw_degrees(5.0) {
        }

        bool load_from_rosparams(const std::shared_ptr<rclcpp::Node> nh);
    };

// todo should be a common representation
    struct XYZYaw {
        double x;
        double y;
        double z;
        double yaw;
    };

// todo should be a common representation
    class DynamicConstraints {
    public:
        double max_vel_horz_abs; // meters/sec
        double max_vel_vert_abs;
        double max_yaw_rate_degree;

        DynamicConstraints()
                : max_vel_horz_abs(1.0), max_vel_vert_abs(0.5), max_yaw_rate_degree(10.0) {
        }
        bool load_from_rosparams(const std::shared_ptr<rclcpp::Node> nh);
    };

    class PDPositionController : public rclcpp::Node {
    private:
        PDControllerParams params_;
        DynamicConstraints constraints_;

        std::string vehicle_name_;
        double control_update_rate_; // [sec]

        // ROS2 Publisher
        rclcpp::Publisher<airsim_interfaces::msg::VelCmd>::SharedPtr airsim_vel_cmd_world_frame_pub_;
        // ROS2 Subscriber
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr airsim_odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr airsim_local_goal_sub_;

        // Callback Functions
        void AirsimOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
        void AirsimLocalGoalCallback(const nav_msgs::msg::Odometry::SharedPtr goal_msg);

        // PD Control Functions
        void ComputeControlCommand();
        void EnforceDynamicConstraints();
        void PublishCommand();
        void ControlTimerCallback();

        RosTimer control_timer_;

        airsim_interfaces::msg::VelCmd vel_cmd_;
        bool has_local_odom_ = false;
        bool has_local_goal_ = false;
        XYZYaw current_pose_;
        XYZYaw previous_error_{0.0,0.0,0.0,0.0};
        XYZYaw current_error_;
        XYZYaw current_goal_pose_;

    public:
        explicit PDPositionController(const rclcpp::NodeOptions &options_input);
    };

}

#endif //AIRSIM_ROS_PKGS_PD_POSITION_CONTROLLER_SIMPLE_TOPIPDC_H
