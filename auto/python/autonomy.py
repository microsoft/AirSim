import rospy
import time
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler
from airsim2ros.msg import Barometer, Distance
from nav_msgs.msg import Odometry
from multiprocessing import Process

from tf.transformations import quaternion_from_matrix, rotation_matrix, quaternion_matrix

from pad_detection import PadDetector
from airsim_helper import AirSimHelper

import setup_path
import airsim
import cv2
import time
import numpy as np
import roslaunch
import atexit
import sys
import tf2_ros
import tf2_geometry_msgs
from threading import Thread

class Autonomy:

	def __init__(self):

		rospy.init_node('autonomy', anonymous=True)

		#Helper function to setup 'True' Drone pose and Video ROS Topics
		#Not intended for actual drone use
		AirSim = AirSimHelper()
		AirSim.daemon = True
		AirSim.start()
		
		rospy.spin()


if __name__ == "__main__":
	Autonomy()


