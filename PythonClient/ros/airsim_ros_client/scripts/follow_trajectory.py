#!/usr/bin/env python

# airsim
import setup_path
import airsim
# standard python
import math
import sys
import numpy as np
# ROS
import rospy
import tf2_ros
# ROS messages
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

velocity = 0.5

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

def trajectory_callback(spline_marker_array_msg):
    # follow the last published trajectory
    last_traj_num = len(spline_marker_array_msg.markers)
    following_path = []    

    global velocity

    for pose in spline_marker_array_msg.markers[last_traj_num-1].points:
	# add point to trajectory queue
        print("[NED] Adding point (%f, %f, %f)" % (pose.x, - pose.y, - pose.z))
	following_path.append(airsim.Vector3r(pose.x, - pose.y, - pose.z))

    client.moveOnPathAsync(following_path, velocity, 3e+38, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), -1, 1).join()

    client.enableApiControl(False)

def airsim_follow():
    ## Start ROS ---------------------------------------------------------------
    rospy.init_node('airsim_follow_trajectory')

    ## Parameters
    global velocity
    velocity = rospy.get_param("/velocity", 0.5)

    ## Subscribers --------------------------------------------------------------
    rospy.Subscriber("/trajectory/spline_marker_array", MarkerArray, trajectory_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        airsim_follow()
    except rospy.ROSInterruptException:
        pass

