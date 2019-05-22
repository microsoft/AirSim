#include "ros/ros.h"
#include "airsim_ros_pkgs/PointArray.h"
#include "common.h"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include <mav_trajectory_generation/trajectory.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "airsim_ethz_asl_adaptor.h"

class TrajGenHelper
{
public:
	TrajGenHelper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
	~TrajGenHelper();
	bool waypoint_cb(const airsim_ros_pkgs::PointArray& waypoints_msg);

private:
	airsim_ros::DynamicConstraintsVelAccYaw dynamic_constraints_;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;
	int derivative_to_optimize_;
	int order_poly_;
	int magic_fabian_constant_;
};