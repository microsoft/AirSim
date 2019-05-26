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


    // todo auto init odom topic. scale to multiple drones in airsim
    odometry_sub_ = nh_.subscribe("/airsim_node/drone_1/odom_ned", 1, &TrajGenHelper::odom_cb, this);
    waypoint_list_sub_ = nh_.subscribe("waypoint_list", 1, &TrajGenHelper::waypt_vec_cb, this);
    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
}

void TrajGenHelper::odom_cb(const nav_msgs::Odometry& msg)
{
    has_odom_ = true;
    curr_odom_ = odom_msg;
}

void TrajGenHelper::clear_traj()
{
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);
    command_publishing_timer_.stop();
    path_queue_.clear();
    path_index_ = 0;
}

void MavLocalPlanner::planningStep()
{
    if (current_waypoint_ < 0 || static_cast<int>(waypoints_.size()) <= current_waypoint_)
        return;

    mav_trajectory_generation::timing::MiniTimer timer;

    mav_msgs::EigenTrajectoryPointVector waypoints;
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    waypoints.push_back(current_point);
    waypoints.insert(waypoints.end(), waypoints_.begin(), waypoints_.end());

    mav_msgs::EigenTrajectoryPointVector path;

    if (planPathThroughWaypoints(waypoints, &path))
    {
        replacePath(path);
        current_waypoint_ = waypoints_.size();
    }
    else
    {
        ROS_ERROR("[Mav Local Planner] Waypoint planning failed!");
    }

}

void TrajGenHelper::waypt_vec_cb(const airsim_ros_pkgs::PointArray& waypoints_msg)
{
    // Plan a path from the current position to the target pose stamped.
    ROS_INFO("[TrajGenHelper] Got a list of waypoints, %zu long!", waypoints_msg.points.size());
    // Cancel any previous trajectory on getting a new one.
    clear_traj();

    waypoints_.clear();

    for (const geometry_msgs::Pose& pose : msg.poses)
    {
        mav_msgs::EigenTrajectoryPoint waypoint;
        eigenTrajectoryPointFromPoseMsg(pose, &waypoint);
        waypoints_.push_back(waypoint);
    }
    current_waypoint_ = 0;

    // Execute one planning step on main thread.
    planningStep();
    startPublishingCommands();
}

bool TrajGenHelper::fit_traj_to_waypoints(const airsim_ros_pkgs::PointArray& waypoints_msg)
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