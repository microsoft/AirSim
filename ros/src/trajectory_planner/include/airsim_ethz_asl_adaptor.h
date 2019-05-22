#include "airsim_ros_pkgs/PointArray.h"
#include "mav_msgs/conversions.h"

namespace airsim_ethz_asl_adaptor
{
	inline mav_msgs::EigenTrajectoryPoint::Vector eigenTrajectoryPointVecFromAirsimMsg(const airsim_ros_pkgs::PointArray& waypoints_msg)
	{
		mav_msgs::EigenTrajectoryPoint::Vector trajectory_point_vec;

		for (const auto& point : waypoints_msg.points)
		{
			mav_msgs::EigenTrajectoryPoint trajectory_point;
			trajectory_point.position_W = mav_msgs::vector3FromPointMsg(point);
			trajectory_point_vec.push_back(trajectory_point);
		}

		return trajectory_point_vec;
	}
};