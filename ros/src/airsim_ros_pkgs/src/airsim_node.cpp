#include "ros/ros.h"
#include "airsim_ros_wrapper.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    AirsimROSWrapper airsim_ros_wrapper(nh, nh_private);

    std::cout << "airsim_ros_wrapper.num_threads_: " << airsim_ros_wrapper.num_threads_ << std::endl;
	ros::MultiThreadedSpinner multi_thread(airsim_ros_wrapper.num_threads_);
	multi_thread.spin(); 

	// ros::AsyncSpinner async_spinner(num_threads);
	// async_spinner.start();

	// single threaded spinner
    // ros::spin();
    return 0;
} 