#include "ros/ros.h"
#include "airsim_ros_pkgs/PointArray.h"
#include "common.h"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include <mav_trajectory_generation/trajectory.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "airsim_ethz_asl_adaptor.h"

// todo make a generic interface to test and benchmark spline fitters / smoothers from various labs 
// current version is adapted from ethz-asl's code
// ref: https://github.com/ethz-asl/mav_trajectory_generation#linear-optimization
// ref: https://github.com/ethz-asl/mav_voxblox_planning's PolynomialSmoother::getTrajectoryBetweenWaypoints()
// ref: https://bitbucket.org/castacks/mav_fcs/src/master/
class TrajGenHelper
{
public:
	TrajGenHelper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
	~TrajGenHelper();
	bool waypoint_cb(const airsim_ros_pkgs::PointArray& waypoints_msg);

private:
	void traj_pub_timer_cb(const ros::TimerEvent& event);

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	ros::CallbackQueue command_publishing_queue_;
	ros::AsyncSpinner command_publishing_spinner_;
	ros::CallbackQueue planning_queue_;
	ros::AsyncSpinner planning_spinner_;

	ros::Subscriber waypoint_sub_;
	ros::Subscriber waypoint_list_sub_;
	ros::Publisher command_pub_;
	ros::Publisher path_marker_pub_;
	ros::Publisher full_trajectory_pub_;

	mav_msgs::EigenTrajectoryPointVector waypoints_;
	int current_waypoint_idx_;

	mav_msgs::EigenTrajectoryPointVector path_queue_;
	size_t path_idx_;
	std::recursive_mutex path_mutex_;

	int max_failures_;
	int num_failures_;

    nav_msgs::Odometry curr_odom_;

	double replan_dt_;
	double replan_lookahead_sec_;

	airsim_ros::DynamicConstraintsVelAccYaw dynamic_constraints_;
	int derivative_to_optimize_;
	int order_poly_;
	int magic_fabian_constant_;
};