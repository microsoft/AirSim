#include "traj_gen_helper.h"

TrajGenHelper::TrajGenHelper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	nh_private_.getParam("derivative_to_optimize", derivative_to_optimize_);
	if (derivative_to_optimize_ == 3) // snap
		order_poly_ = 10;

	if (derivative_to_optimize_ == 2) // jerk
		order_poly_ = 8;

	dynamic_constraints_.load_from_rosparams(nh_);
	magic_fabian_constant_ = 6.5; // in hardcoded magic, we trust
}

// ref: https://github.com/ethz-asl/mav_trajectory_generation#linear-optimization
// mav_voxblox_planning's PolynomialSmoother::getTrajectoryBetweenWaypoints()
bool TrajGenHelper::waypoint_cb(const airsim_ros_pkgs::PointArray& waypoints_msg)
{
	const mav_msgs::EigenTrajectoryPoint::Vector waypoints = airsim_ethz_asl_adaptor::eigenTrajectoryPointVecFromAirsimMsg(waypoints_msg);
	const int dimension = 3; // todo param

	mav_trajectory_generation::Vertex::Vector vertices;
	int counter = 0;
	for (auto& curr_waypt : waypoints)
	{
		mav_trajectory_generation::Vertex vertex(dimension);

		// if first or last waypoint
		if ((counter == 0) or (counter == waypoints.size()-1))
		{
			vertex.makeStartOrEnd(0, derivative_to_optimize_);
			vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curr_waypt.position_W);
		}
		// if in the middle, add a position constraint for solver
		// todo support higher order constraints
		else
		{
			vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, curr_waypt.position_W);  
		}
		vertices.push_back(vertex);
	}

	// todo separate horz and vert velocity limits? 
	// std::vector<double> segment_times = estimateSegmentTimes(vertices, min(dynamic_constraints_.max_vel_horz_abs, dynamic_constraints_.min_vel_horz_abs), 
											// dynamic_constraints_.max_acceleration, magic_fabian_constant_);
	// todo last parameter is unused in mav_trajectory_generation::estimateSegmentTimesVelocityRamp()
  	std::vector<double> segment_times = mav_trajectory_generation::estimateSegmentTimesVelocityRamp(vertices, dynamic_constraints_.max_vel, dynamic_constraints_.max_acc, 1.0);

  	constexpr int order_poly_const = 10; // todo 
	mav_trajectory_generation::PolynomialOptimization<order_poly_const> opt(dimension);
	opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
	mav_trajectory_generation::Trajectory* trajectory;
	if (opt.solveLinear())
	{
		opt.getTrajectory(trajectory);
	}
	else
	{
		// ROS_INFO_STREAM("")
		return false;
	}

	// mav_trajectory_generation::Segment::Vector segments;
	// opt.getSegments(&segments);

	double max_vel, max_acc;

	trajectory->computeMaxVelocityAndAcceleration(&max_vel, &max_acc);
	ROS_INFO("[pre time scaling] V max/limit: %f/%f, A max/limit: %f/%f", max_vel, dynamic_constraints_.max_vel, max_acc, dynamic_constraints_.max_acc);

	trajectory->computeMaxVelocityAndAcceleration(&max_vel, &max_acc);
	trajectory->scaleSegmentTimesToMeetConstraints(dynamic_constraints_.max_vel, dynamic_constraints_.max_acc);
	ROS_INFO("[post time scaling] V max/limit: %f/%f, A max/limit: %f/%f", max_vel, dynamic_constraints_.max_vel, max_acc, dynamic_constraints_.max_acc);
	return true;
}