#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace utils
{
	double get_yaw_from_quat_msg(const geometry_msgs::Quaternion& quat_msg)
	{
		tf2::Quaternion quat_tf;
		double roll, pitch, yaw;
		tf2::fromMsg(quat_msg, quat_tf);
	    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
	    return yaw;
	}

	// OdometryEuler get_eigen_odom_from_rosmsg(const nav::msgs &odom_msg);
}