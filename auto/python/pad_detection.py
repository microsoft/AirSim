import rospy
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseWithCovarianceStamped, PoseStamped, Point
from cv_bridge import CvBridge
from airsim2ros.msg import Barometer, Distance
from nav_msgs.msg import Odometry
from multiprocessing import Process
from tf import TransformListener

from cv_bridge import CvBridge, CvBridgeError

from tf.transformations import quaternion_from_matrix, rotation_matrix, euler_from_quaternion, quaternion_multiply

import setup_path
import airsim
import cv2
import math
import numpy as np
import roslaunch
import tf2_ros
import tf2_geometry_msgs
from matplotlib import pyplot as plt
import threading

MIN_MATCH_COUNT = 10

BOARD_SIZE = 1.00

class PadDetector():

	# Define a callback for the Image message
	def imageCallback(self, img_msg):
		self.curr_time = img_msg.header.stamp
		# Try to convert the ROS Image message to a CV2 Image
		try:
			self.curr_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
		except CvBridgeError, e:
			rospy.logerr("CvBridge Error: {0}".format(e))

	def camInfoCallback(self, cam_info):
		# Try to convert the ROS Image message to a CV2 Image
		self.K = np.reshape(cam_info.K,(3,3)).astype(np.float32)
		self.D = np.reshape(cam_info.D,(1,5)).astype(np.float32)

	def buildPoseMsg(self, rvec, tvec):

		pose_msg = PoseStamped()
		pose_msg.header.stamp = self.curr_time
		pose_msg.header.frame_id = 'Drone1'
		#get 3x3 mat
		rmat, _jac = cv2.Rodrigues(rvec)

		#transform board frame to camera frame
		R = rmat.transpose()
		t = -R.dot(tvec)

		#Switch x and y coordinates going from image to camera frame?
		t = np.array([t[1]-BOARD_SIZE/2,-t[0]+BOARD_SIZE/2,-t[2]])
		P = np.hstack((R,t))

		#Transform from camera frame to REP-103
		cam_2_ned = np.array(	[[1,0,0],
					 [0,1,0],
					 [0,0,-1]])
		P = cam_2_ned.dot(P)
		R = P[0:3,0:3]
		R = np.vstack((R,[0,0,0]))
		R = np.hstack((R,[[0],[0],[0],[1]]))
		t = P[0:3,3]
		#convert to quaternion and build msg
		q = quaternion_from_matrix(R)

		pose_msg.pose.position = Point(t[0],t[1],t[2])
		pose_msg.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

		return pose_msg		

	# Override the run() function of Thread class
	def run(self):
		rate = rospy.Rate(30)  # 30hz
		# Load the predefined dictionary
		dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
		# Generate the board
		board = cv2.aruco.GridBoard_create(2,2, 0.46, 0.08, dictionary)
		# Initialize the detector parameters using default values
		parameters =  cv2.aruco.DetectorParameters_create()
		while not rospy.is_shutdown():
			img1 = self.curr_image;
			# Detect the markers in the image
			markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img1, dictionary, parameters=parameters)
			try:
				_ret, rvec, tvec = cv2.aruco.estimatePoseBoard(markerCorners, markerIds, board, self.K, self.D)
				self.pose_pub.publish(self.buildPoseMsg(rvec,tvec))
			except:
				continue
			rate.sleep()

	def stopped(self):
        	return self._stop_event.is_set()

	def __init__(self):
		rospy.init_node('autonomy', anonymous=True)
		rospy.Subscriber('/airsim/Drone1/main/image_raw', Image, self.imageCallback)
		rospy.Subscriber('/airsim/Drone1/main', CameraInfo, self.camInfoCallback)
       		self._stop_event = threading.Event()

		rate = rospy.Rate(30)  # 30hz
		self.curr_image = np.array([])
		self.curr_time = 0
		self.bridge = CvBridge()
		self.K = np.array([])
		self.D = np.array([])
		self.transformer = TransformListener()
		self.tfBuffer = tf2_ros.Buffer()
    		listener = tf2_ros.TransformListener(self.tfBuffer)

		self.pose_pub = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)

		if (len(self.curr_image) == 0):
			rospy.wait_for_message('/airsim/Drone1/main/image_raw',Image)
		print('got image')

		if (len(self.K) == 0):
			rospy.wait_for_message('/airsim/Drone1/main',CameraInfo)
		print('got camInfo')


if __name__ == "__main__":
	pd = PadDetector()
	pd.run()




