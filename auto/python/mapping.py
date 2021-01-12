import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
import math
import struct

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

from sfm import getColors
from sfm import triangulate
from sfm import triangulate_int
from sfm import drawTracks
from sfm import getTrackLength
from sfm import getObjectPointsEssential
from sfm import eliminateDuplicateObjects
from sfm import baFun
from sfm import bundle_adjustment_sparsity

from scipy.sparse import lil_matrix
from scipy.optimize import least_squares

from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_multiply

class mapping():

	def __init__(self):

   		rospy.init_node('mapping', anonymous=True)

		self.bridge = CvBridge()
		self.tfBuffer = tf2_ros.Buffer()
    		listener = tf2_ros.TransformListener(self.tfBuffer)

		self.image = []
		self.pose = PoseStamped()
		self.K = []
		self.d = []
		self.cam_width =  []
		self.cam_height = []
		self.rotation = []
		self.translation = []
		self.tracking = False

		self.img_curr = []
		self.img_prev = []

		self.features_orig = []
		self.features_prev = []

		self.obj_mask = []

		self.reset = True

		#INITIALIZE FEATURE MATCHING PARAMETERS#
		self.maxCorners = 500 #Maximum number of corners to return. If there are more corners than are found, the strongest of them is returned
		self.qualityLevel = 0.01 #For example, if best corner has measure = 1500, and qualityLevel=0.01 , then corners with quality<15 are rejected.
		self.minDistance = 10 #Minimum possible Euclidean distance between the returned corners.
		self.lk_params = dict(winSize = (15,15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

		#rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
		rospy.Subscriber('orb_slam2_mono/pose', PoseStamped, self.poseCallback)
		rospy.Subscriber('/airsim/base_link/camera/image_raw', Image, self.imageCallback)
		rospy.Subscriber('/airsim/base_link/camera', CameraInfo, self.camInfoCallback)
		self.cloud_pub = rospy.Publisher("cloud", PointCloud2, queue_size=10)
		self.test_pose = rospy.Publisher("test_pose", PoseStamped, queue_size=10)

		print('waiting on topics...')
		rospy.wait_for_message('/airsim/base_link/camera', Image)
		#self.cam_width = self.img_curr.shape[0]
		#self.cam_height = self.img_curr.shape[1]
		print('K: ', self.K)
		print('D: ', self.D)
		rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
		print('connected')

		rospy.spin()

	def poseCallback(self, data):
		pose_stamped = PoseStamped()
		pose_stamped.pose = data.pose
		pose_stamped.header = data.header
		self.pose.header = pose_stamped.header
		self.pose = data
		try:
			t = self.tfBuffer.lookup_transform('camera_pose','base_link',rospy.Time(0),rospy.Duration(1.0))
			#self.pose = tf2_geometry_msgs.do_transform_pose(self.pose, t)
			#self.pose.pose.position.x = pose_stamped.pose.position.x + t.transform.translation.x
			#self.pose.pose.position.y = pose_stamped.pose.position.y + t.transform.translation.y
			#self.pose.pose.position.z = pose_stamped.pose.position.z + t.transform.translation.z
			#q0 = (pose_stamped.pose.orientation.x,pose_stamped.pose.orientation.y,pose_stamped.pose.orientation.z,pose_stamped.pose.orientation.w)
			#q1 = (self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w)
			#q = quaternion_multiply(q1,q0)
			#self.pose.pose.orientation.x = q[0]
			#self.pose.pose.orientation.y = q[1]
			#self.pose.pose.orientation.z = q[2]
			#self.pose.pose.orientation.w = q[3]
			self.test_pose.publish(self.pose)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print('Error loading gimbal transform')
		q = (self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w)
		r = quaternion_matrix(q)
		self.rotation = r[0:3,0:3]
		self.translation = np.array(([self.pose.pose.position.x],[self.pose.pose.position.y],[self.pose.pose.position.z]))
		#print(self.translation)
		
	def imageCallback(self, data):
		self.header = data.header
		self.img_curr = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

		P1 = np.hstack((self.rotation,self.translation))

		grey_curr = cv2.cvtColor(self.img_curr, cv2.COLOR_RGB2GRAY)

		#check if we are currently tracking flow, if no, we need to initialize and wait for next image
		if (self.reset or ):
			#Get first set of features to track
			self.features_orig = cv2.goodFeaturesToTrack(grey_curr, self.maxCorners, self.qualityLevel, self.minDistance)
			self.features_prev = self.features_orig
			self.P0 = P1
			self.obj_mask = np.full((len(self.features_orig), 4),-1)
			self.reset = False
			self.img_prev = self.img_curr
			return

		grey_prev = cv2.cvtColor(self.img_prev, cv2.COLOR_RGB2GRAY)

		#GET NEW FEATURE LOCATIONS#
		next, status, error = cv2.calcOpticalFlowPyrLK(grey_prev, grey_curr , self.features_prev, None, **self.lk_params)
		if (not next.any()):
			self.reset = True
			return

		#Filter out the points with high error 
		error = error[status[:,0] == 1]
		new = next[status[:,0] == 1]
		new = new[error[:,0] < 10]

		#Update the original list of features to reflect pixels that have been lost in the flow
		self.features_orig = self.features_orig[status[:,0] == 1]
		self.features_orig = self.features_orig[error[:,0] < 10]

		# Updates previous good feature points
		self.features_prev = new.reshape(-1, 1, 2) 

		#Optional visualization
		output = drawTracks(self.features_orig.astype(int), new, self.img_curr, (0, 255, 0))
		cv2.imshow('Tracks',output)
		self.img_prev = self.img_curr
		key = cv2.waitKey(1) & 0xFF;

		#Check sufficient parallax# 
		avg_track_len = getTrackLength(self.features_orig,new)
		if (avg_track_len < 30):
			#not enough parrallax
			return

		#Attempt to calculate SFM
		orig_pts = self.features_orig.reshape(len(self.features_orig),2)

		new_pts = new.reshape(len(new),2)
		mask_inf = np.zeros(len(new_pts))

		#TODO filter out points with low parallax

		print(len(orig_pts))
		#Check quality of pts
		if (len(orig_pts) < 10):
			#Not enough quality points, reset
			self.reset = True
			return

		#SolvePNP
		#Get 3d world points and associated pixel values for the second image
		print(np.reshape(self.P0[:,3],(1,3)),np.reshape(P1[:,2],(1,3)))
		print(P1,(self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z))
		obj_pts, prev_pts, new_pts, P = getObjectPointsEssential(orig_pts,new_pts,self.P0,self.K, self.D)

        	#obj_pts, mask_tri, error = triangulate(orig_pts, self.P0, new_pts, P1, self.K, self.D)
		
		#Get colors from detected pixels for coloring pointcloud

		#pixel_pts = new_pts[mask_tri[:,0]==1]
		colors = getColors(new_pts,self.img_curr.copy())		

		self.publishPoints(obj_pts, colors)
		self.reset = True

		return

	def camInfoCallback(self,data):
		self.K = np.reshape(data.K,(3,3))
		self.D = np.reshape(data.D,(1,5))

	def publishPoints(self, obj_pts, colors):
		points = []		
		for i in range(len(obj_pts)):
			x = obj_pts[i,0]
			y = obj_pts[i,1]
			z = obj_pts[i,2]
			#print(obj_pts[i])
			r = colors[i,0]
			g = colors[i,1]
			b = colors[i,2]
			a = 255
			rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
			pt = [x, y, z, rgb]			
			points.append(pt)
		fields = [PointField('x', 0, PointField.FLOAT32, 1),
			  PointField('y', 4, PointField.FLOAT32, 1),
			  PointField('z', 8, PointField.FLOAT32, 1),
			  PointField('rgba', 12, PointField.UINT32, 1),
			  ]
		#Raytracing (clearing of free space) is always done from the origin of this frame_id at the time of the point cloud
		header = self.header
		header.frame_id = 'map'
		cloud = point_cloud2.create_cloud(header, fields, points)
		self.cloud_pub.publish(cloud)

if __name__ == '__main__':
	mapping()
