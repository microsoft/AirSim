import rospy
import glob
import json
import math
import os
import px4tools
import sys
import time
import actionlib
import roslaunch
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import cv2
import shutil

from cv_bridge import CvBridge, CvBridgeError
from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached, GlobalPositionTarget, State, TakeoffAction, TakeoffGoal, LandAction, LandGoal, WaypointsAction, WaypointsGoal, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear, CommandHome
from orb_slam2_ros.msg import KeyFrames, Observations
from sensor_msgs.msg import NavSatFix, Image, PointCloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseStamped, PoseWithCovarianceStamped
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil

class KeyFrame(object):
	empty = []

	def __init__(self):
		self.image_id = ""
		self.latitude = 0.0
		self.longitude = 0.0
		self.altitude = 0.0
		self.orientation = [0.0,0.0,0.0,0.0]
		self.image = ""	

class companion():

	def __init__(self):

		#Initialize ROSLaunch
		launch = roslaunch.scriptapi.ROSLaunch()
		launch.start()	
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)

		#Launch MAVROS
		cli_args = ('mavros', 'px4.launch')
		roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0])]
		mavros = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
		mavros.start()

   		rospy.init_node('companion_node', anonymous=True)
		
		#clear old images
		shutil.rmtree('img')
		os.mkdir('img')		

		self.local_pose = Pose()
		self.slam_pose = PoseStamped()
		self.raw_slam_pose = PoseStamped()
		self.calib_init_slam = Pose()
		self.calib_init_local = Pose()
		self.slam_calibrated = False
		self.slam_offset_x = 0
		self.slam_offset_y = 0
		self.slam_offset_z = 0
		self.slam_scale = 1
		self.state = "SYSTEM_NOT_READY"
		self.last_state = "SYSTEM_NOT_READY"

			#States:
			#"SYSTEM_NOT_READY"
			#"NO_IMAGES_YET"
			#"NOT_INITIALIZED"
			#"OK"
			#"LOST"

		self.keyframes = []
		self.image_buffer = []
		self.bridge = CvBridge()

		rospy.spin()

		#Code to scale and publish orbslam pose to px		
		
if __name__ == '__main__':
	companion()
