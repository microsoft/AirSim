#ifndef _PID_POSITION_CONTROLLER_SIMPLE_H_
#define _PID_POSITION_CONTROLLER_SIMPLE_H_

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF //todo what does this do?
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

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

// todo nicer api
class PIDParams
{
public:
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_yaw;
    double kd_x;
    double kd_y;
    double kd_z;
    double kd_yaw;

    double reached_thresh_xyz;
    double reached_yaw_degrees;

    PIDParams():
        kp_x(0.5),
        kp_y(0.5),
        kp_z(0.5),
        kp_yaw(0.5),
        kd_x(0.1),
        kd_y(0.1),
        kd_z(0.1),
        kd_yaw(0.1),
        reached_thresh_xyz(0.5),
        reached_yaw_degrees(5.0)
        {}

    bool load_from_rosparams(const ros::NodeHandle& nh);
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
class DynamicConstraints
{
public:
    double max_vel_horz_abs; // meters/sec
    double max_vel_vert_abs;
    double max_yaw_rate_degree;

    DynamicConstraints():
        max_vel_horz_abs(1.0),
        max_vel_vert_abs(0.5),
        max_yaw_rate_degree(10.0)
        {}

    bool load_from_rosparams(const ros::NodeHandle& nh);
};

class PIDPositionController
{
public:
    PIDPositionController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    // ROS service callbacks
    bool local_position_goal_srv_cb(airsim_ros_pkgs::SetLocalPosition::Request& request, airsim_ros_pkgs::SetLocalPosition::Response& response); 
    bool local_position_goal_srv_override_cb(airsim_ros_pkgs::SetLocalPosition::Request& request, airsim_ros_pkgs::SetLocalPosition::Response& response); 
    bool gps_goal_srv_cb(airsim_ros_pkgs::SetGPSPosition::Request& request, airsim_ros_pkgs::SetGPSPosition::Response& response);
    bool gps_goal_srv_override_cb(airsim_ros_pkgs::SetGPSPosition::Request& request, airsim_ros_pkgs::SetGPSPosition::Response& response);

    // ROS subscriber callbacks
    void airsim_odom_cb(const nav_msgs::Odometry& odom_msg);
    void home_geopoint_cb(const airsim_ros_pkgs::GPSYaw& gps_msg);

    void update_control_cmd_timer_cb(const ros::TimerEvent& event);

    void reset_errors();

    void initialize_ros();
    void compute_control_cmd();
    void enforce_dynamic_constraints();
    void publish_control_cmd();
    void check_reached_goal();

private:
    geodetic_converter::GeodeticConverter geodetic_converter_;
    bool use_eth_lib_for_geodetic_conv_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    DynamicConstraints constraints_;
    PIDParams params_;
    XYZYaw target_position_;
    XYZYaw curr_position_;
    XYZYaw prev_error_;
    XYZYaw curr_error_;

    bool has_home_geo_;
    airsim_ros_pkgs::GPSYaw gps_home_msg_;

    nav_msgs::Odometry curr_odom_;
    airsim_ros_pkgs::VelCmd vel_cmd_;
    bool reached_goal_;
    bool has_goal_;
    bool has_odom_;
    bool got_goal_once_;
    // todo check for odom msg being older than n sec

    ros::Publisher airsim_vel_cmd_world_frame_pub_;
    ros::Subscriber airsim_odom_sub_;
    ros::Subscriber home_geopoint_sub_;
    ros::ServiceServer local_position_goal_srvr_;
    ros::ServiceServer local_position_goal_override_srvr_;
    ros::ServiceServer gps_goal_srvr_;
    ros::ServiceServer gps_goal_override_srvr_;

    ros::Timer update_control_cmd_timer_;
};

#endif /* _PID_POSITION_CONTROLLER_SIMPLE_ */