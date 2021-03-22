#!/usr/bin/env python

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2

import struct

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image,CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header

from cv_bridge import CvBridge, CvBridgeError

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

class Stereo():

	def depthCallback(self,data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def colorCallback(self,data):
		try:
			self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

	def infoCallback(self, data):
		self.img_height = data.height
		self.img_width = data.width
		self.frame = data.header.frame_id
		self.time = data.header.stamp
		self.D = np.reshape(data.D,(1,5))
		self.K = np.reshape(data.K,(3,3))

	def array2pointcloud(self, xyz_pts, colors, stamp=None, frame_id=None):
		points = []
		for i in range(xyz_pts.shape[0]):
		    rgb = struct.unpack('I', struct.pack('BBBB', colors[i,0], colors[i,1], colors[i,2], 255))[0]
		    pt = [xyz_pts[i,0],xyz_pts[i,1],xyz_pts[i,2], rgb]
		    points.append(pt)

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
			  PointField('y', 4, PointField.FLOAT32, 1),
			  PointField('z', 8, PointField.FLOAT32, 1),
			  # PointField('rgb', 12, PointField.UINT32, 1),
			  PointField('rgba', 12, PointField.UINT32, 1),
			  ]

		header = Header()
		header.frame_id = frame_id
		header.stamp = stamp
		pc2 = point_cloud2.create_cloud(header, fields, points)
		self.pc_pub.publish(pc2)


	def __init__(self):

		rospy.init_node('stereo', anonymous=True)

		self.bridge = CvBridge()
		self.color_sub = rospy.Subscriber("airsim/Drone1/color/image_raw",Image,self.colorCallback)
		self.depth_sub = rospy.Subscriber("airsim/Drone1/depth/image_raw",Image,self.depthCallback)
		self.info_sub = rospy.Subscriber("airsim/Drone1/depth",CameraInfo,self.infoCallback)
		self.pc_pub = rospy.Publisher("airsim/Drone1/stereo", PointCloud2, queue_size=2)
		self.color_image = np.array(0)
		self.depth_image = np.array(0)
		self.img_height = 0
		self.img_width = 0
		self.time = 0
		self.frame = ""
		self.D = np.array(0)
		self.K = np.array(0)

		rospy.wait_for_message('/airsim/Drone1/depth/image_raw',Image)
		print('got depth image')

		rospy.wait_for_message('/airsim/Drone1/color/image_raw',Image)
		print('got color image')

		rospy.wait_for_message('/airsim/Drone1/depth', CameraInfo)
		print('got camera info')
		
		rate = rospy.Rate(10)  # 30hz
		h, w = self.img_height, self.img_width
		f = self.K[0,0]          #focal length  

		Q = np.array([	[0,-1,0, -320],
				[1,0,0, -240],
				[0,0,1, 204],
				[0,0,33, 0]])

		while(True):

			disp = np.array(cv2.cvtColor(self.depth_image, cv2.COLOR_BGR2GRAY)).astype(np.float32) / 16.0
			points = cv2.reprojectImageTo3D(disp, Q)
			colors = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
			mask = disp > disp.min()
			out_points = points[mask]
			out_colors = colors[mask]

			self.array2pointcloud(out_points, out_colors, self.time, self.frame)

			#cv2.imshow('left', self.left_image)
			cv2.imshow('disparity', (disp))
			cv2.waitKey(1)
			rate.sleep()


if __name__ == '__main__':
	Stereo()
	exit()

