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

from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached, GlobalPositionTarget, State, TakeoffAction, TakeoffGoal, LandAction, LandGoal, WaypointsAction, WaypointsGoal, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear, CommandHome
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseStamped, PoseWithCovarianceStamped
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil

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

		#Launch ORBSLAM
		cli_args = ('orb_slam2_ros', 'orb_slam2_mono.launch')
		roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0])]
		orbslam = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
		orbslam.start()

   		rospy.init_node('companion_node', anonymous=True)

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

		self.timer = 0
		self.calibration_check_interval = 128 #in px4 freq

		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pose_callback)
		rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slamCallback)
		self.slam_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=50)

		self.tfBuffer = tf2_ros.Buffer()
    		listener = tf2_ros.TransformListener(self.tfBuffer)

		self.cam_t = self.tfBuffer.lookup_transform('base_link','camera_pose',rospy.Time(0),rospy.Duration(1.0))

		rospy.spin()

		#Code to scale and publish orbslam pose to px4

	def local_pose_callback(self,data):
		self.local_pose = data.pose
		self.timer
		if (self.timer == self.calibration_check_interval):
			self.timer = 0
			self.runCalibration()
		else:
			self.timer += 1
		
	def slamCallback(self,data):
		pose_converted = self.convertFRD(data)
		
		if (self.raw_slam_pose.header.stamp.secs < pose_converted.header.stamp.secs - 1) or (self.calib_init_slam.position.x == 0) :
			self.calib_init_slam = pose_converted.pose
			self.calib_init_local = self.local_pose
			print(self.calib_init_slam.position,self.calib_init_local.position)
			self.slam_calibrated = False

		self.raw_slam_pose = pose_converted
		if (self.slam_calibrated):

			#print('bfore: ', self.raw_slam_pose.pose.position.x,self.raw_slam_pose.pose.position.y,self.raw_slam_pose.pose.position.z)
			#transform SLAM pose
			self.slam_pose.pose.position.x = self.raw_slam_pose.pose.position.x * self.slam_scale + self.slam_offset_x + self.cam_t.transform.translation.x
			self.slam_pose.pose.position.y = self.raw_slam_pose.pose.position.y * self.slam_scale + self.slam_offset_y + self.cam_t.transform.translation.y
			self.slam_pose.pose.position.z = self.raw_slam_pose.pose.position.z * self.slam_scale + self.slam_offset_z + self.cam_t.transform.translation.z

			self.slam_pose.pose.orientation = self.raw_slam_pose.pose.orientation
			
			self.slam_pose.header = self.raw_slam_pose.header

			self.slam_pub.publish(self.slam_pose)
		else:
			self.runCalibration()

	def runCalibration(self):
			#check for sufficient calibration movement
			#TODO Check data timeliness
			x = self.local_pose.position.x - self.calib_init_local.position.x
			y = self.local_pose.position.y - self.calib_init_local.position.y
			x_slam = self.raw_slam_pose.pose.position.x - self.calib_init_slam.position.x
			y_slam = self.raw_slam_pose.pose.position.y - self.calib_init_slam.position.y
			if (math.sqrt(x*x+y*y) > 5.0):
				self.slam_scale = math.sqrt(x*x+y*y)/math.sqrt(x_slam*x_slam+y_slam*y_slam)
				print('scale', self.slam_scale)				
				self.slam_offset_x = (self.calib_init_local.position.x-self.slam_scale*self.calib_init_slam.position.x)
				self.slam_offset_y = (self.calib_init_local.position.y-self.slam_scale*self.calib_init_slam.position.y)
				self.slam_offset_z = (self.calib_init_local.position.z-self.slam_scale*self.calib_init_slam.position.z)
				self.slam_calibrated = True

	def convertFRD(self, data):

		temp = data
		#data.pose.position.x = temp.pose.position.z
		#data.pose.position.y = temp.pose.position.x
		#data.pose.position.z = temp.pose.position.y

		#apply transform from camera pose to robot frame
		transform = self.cam_t
		transform.transform.translation.x = 0.0
		transform.transform.translation.y = 0.0
		transform.transform.translation.z = 0.0
		data = tf2_geometry_msgs.do_transform_pose(data, transform)

		#convert to px4 coord system  (this is working Dec 15th)
		converted = PoseStamped()
		converted.header.frame_id = 'base_link'
		converted.pose.position.x = -data.pose.position.y
		converted.pose.position.y = data.pose.position.x
		converted.pose.position.z = data.pose.position.z
		
		converted.pose.orientation.x = -data.pose.orientation.y
		converted.pose.orientation.y = data.pose.orientation.x
		converted.pose.orientation.z = data.pose.orientation.z
		converted.pose.orientation.w = data.pose.orientation.w
		#print(converted.pose.orientation)
		
		#return data
		return converted


if __name__ == '__main__':
	companion()

