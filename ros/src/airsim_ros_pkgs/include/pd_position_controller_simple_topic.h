//
// Created by larr-lyw on 21. 8. 11..
//

#ifndef AIRSIM_ROS_PKGS_PD_POSITION_CONTROLLER_SIMPLE_TOPIC_H
#define AIRSIM_ROS_PKGS_PD_POSITION_CONTROLLER_SIMPLE_TOPIC_H
#endif
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif //AIRSIM_ROS_PKGS_PD_POSITION_CONTROLLER_SIMPLE_TOPIC_H
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include <string>
#include <iostream>
#include <fstream>
#include <chrono>
#include "common/common_utils/FileSystem.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <airsim_ros_pkgs/SetGPSPosition.h>
#include <airsim_ros_pkgs/GPSYaw.h>
#include <geodetic_conv.hpp>
#include <math_common.h>
#include <utils.h>




class ControllerParams{
public:
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_yaw;

    double kd_x;
    double kd_y;
    double kd_z;
    double kd_yaw;

    double max_vel_horz_abs; // meters/sec
    double max_vel_vert_abs;
    double max_yaw_rate_degree;
    double smoothing_weight;
    std::string vehicle_name;
    std::string world_frame_id;
    std::string odom_frame_id;
    std::string goal_frame_id;

    ControllerParams(): kp_x(1.0),kp_y(1.0),kp_z(1.0),kp_yaw(1.0),kd_x(0.05),kd_y(0.05),kd_z(0.05),kd_yaw(0.05),max_vel_horz_abs(1.0),max_vel_vert_abs(1.0),max_yaw_rate_degree(10.0),vehicle_name("drone")
    {}
};
// todo should be a common representation
struct XYZYaw
        {
    double x;
    double y;
    double z;
    double yaw;
        };

// todo should be a common representation

class PDController{
private:
    ros::NodeHandle nh;
    ControllerParams params_;
    double control_update_rate; // /sec

    //ROS PUB/SUB
    ros::Publisher airsim_vel_cmd_world_frame_pub_;
    ros::Subscriber airsim_odom_sub_;
    ros::Subscriber airsim_local_goal_sub_;



    airsim_ros_pkgs::VelCmd vel_cmd_;
    XYZYaw curr_position_;
    XYZYaw curr_goal_position_;

    XYZYaw prev_error_;
    XYZYaw curr_error_;

    //nav_msgs::Odometry curr_odom_;
    //nav_msgs::Odometry curr_local_goal_;



    bool has_odom_;
    bool has_local_goal_;

public:
    PDController();
    void airsim_odom_cb(const nav_msgs::Odometry& odom_msg);
    void airsim_local_goal_cb(const nav_msgs::Odometry& odom_msg);
    void compute_control_cmd();
    void enforce_dynamic_constraints();
    void prepareROSmsgs();
    void calCmd();
    void publish_cmd();
    void run();

};
