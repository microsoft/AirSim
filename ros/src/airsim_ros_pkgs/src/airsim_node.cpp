#include "ros/ros.h"
#include "airsim_ros_wrapper.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    AirsimROSWrapper airsim_ros_wrapper(nh, nh_private);

	int num_threads = 4;
	ros::MultiThreadedSpinner multi_thread(num_threads);
	multi_thread.spin(); 

	// ros::AsyncSpinner async_spinner(num_threads);
	// async_spinner.start();

	// single threaded spinner
    // ros::spin();
    return 0;
}