import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
import math
import struct

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, NavSatFix 
from sensor_msgs import point_cloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Point
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

from tf.transformations import quaternion_matrix, euler_from_quaternion, quaternion_multiply, quaternion_from_matrix

bowExtractor = cv2.BOWImgDescriptorExtractor()
orb = cv2.ORB_create()

class frame():

	def __init__(self, image):
		self.image = image
		kp = orb.detect(self.image, None)
		kp, desc = orb.compute(self.image,kp)
		
	def computeBOW(self):
		self.BoW = bowExtractor.compute(self.image, kps, desc)

class mapping():

	def gpsCallback(self, data):
		if (not self.gpsInitialized):
			self.initialGPS = data
			self.gpsInitialized = True
		else:
			self.curr_pose.header = data.header
			self.curr_pose.frame_id = "map"
			self.curr_pose.position = self.ecef_to_enu(self.gps_to_ecef(data),self.gps_to_ecef(self.initialGPS))
			self.test_pose.publish(self.curr_pose)

	def gps_to_ecef(self, gps):
		lat = data.latitude
		lon = data.longitude
		alt = data.altitude
		rad_lat = lat * (math.pi / 180.0)
		rad_lon = lon * (math.pi / 180.0)

		a = 6378137.0
		finv = 298.257223563
		f = 1 / finv
		e2 = 1 - (1 - f) * (1 - f)
		v = a / math.sqrt(1 - e2 * math.sin(rad_lat) * math.sin(rad_lat))

		x = (v + alt) * math.cos(rad_lat) * math.cos(rad_lon)
		y = (v + alt) * math.cos(rad_lat) * math.sin(rad_lon)
		z = (v * (1 - e2) + alt) * math.sin(rad_lat)

		position = Point()
		position.x = x
		position.y = y
		position.z = z

		return position

	def ecef_to_enu(self, point, origin):
		enu = Point()
		enu.x = point.x - origin.x
		enu.y = point.y - origin.y
		enu.z = point.z - origin.z
		return enu

	def imageCallback(self, data):
		self.header = data.header
		self.img_curr = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

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

	def __init__(self):

   		rospy.init_node('mapping', anonymous=True)

		self.bridge = CvBridge()
		self.tfBuffer = tf2_ros.Buffer()
    		listener = tf2_ros.TransformListener(self.tfBuffer)

		self.img_curr = []
		self.curr_pose = PoseStamped()
		self.K = []
		self.d = []

		self.gpsInitialized = False
		self.initialGPS = NavSatFix()

		rospy.Subscriber('/mavros/global_position/global', NavSatFix , self.gpsCallback)
		rospy.Subscriber('/airsim/base_link/camera/image_raw', Image, self.imageCallback)
		rospy.Subscriber('/airsim/base_link/camera', CameraInfo, self.camInfoCallback)
		self.cloud_pub = rospy.Publisher("cloud", PointCloud2, queue_size=10)
		self.test_pose = rospy.Publisher("test_pose", PoseStamped, queue_size=10)

		print('waiting on topics...')
		rospy.wait_for_message('/airsim/base_link/camera/image_raw', Image)
		print('K: ', self.K)
		print('D: ', self.D)
		rospy.wait_for_message('/mavros/global_position/global', PoseStamped)
		
		self.run()

	def run(self):

		
		if (self.state != "initialized"):
			self.initialize()

	def initialize(self):
		
		


if __name__ == '__main__':
	mapping()


