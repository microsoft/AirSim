import setup_path
import airsim
import cv2
import json
import platform
import rospy
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
import atexit
import time
import threading
import subprocess
import os
import roslaunch
from os import environ
from os.path import expanduser
from sensor_msgs.msg import Image,CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseStamped, Point
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_matrix, quaternion_from_euler, euler_from_quaternion, quaternion_conjugate

class AirSimHelper(threading.Thread):

	def severConnection(self,ASP):
		for robot in ASP.robots:
			ASP.client.cancelLastTask(robot.name)
			ASP.client.reset()
			if (not robot.sitl):
				robot.px4.communicate('shutdown')
				print('waiting for '+robot.name+' px4_sitl to shutdown')
			print('Severed connection with '+robot.name)
		print('waiting for all processes to finish...')
		time.sleep(1)

	def __init__(self):
		if (platform.system() == "Windows"):
			home = environ['USERPROFILE']
			settings_path = home + "Documents\AirSim\settings.json"
		else:
			home = expanduser("~")
			settings_path = home + "/Documents/AirSim/settings.json"
		with open(settings_path) as f:
			airsim_settings = json.load(f)

       		threading.Thread.__init__(self)

		#initialize the airsim wrapper
		ASP = AirSimPublisher()
		atexit.register(self.severConnection, ASP=ASP)
		ASP.client.confirmConnection()
		

		launch = roslaunch.scriptapi.ROSLaunch()
		launch.start()	
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)

		#Init Rospy and TF
		#rospy.init_node('airsim_publisher', anonymous=True)
		static_br = tf2_ros.StaticTransformBroadcaster()

		vehicles = airsim_settings['Vehicles'].keys()
		for vehicle in vehicles:

			robot = Robot(vehicle)

			cameras = airsim_settings['Vehicles'][vehicle]['Cameras']
			robot.addCameras(cameras)
			try:
				robot.setAddress(airsim_settings['Vehicles'][vehicle]['LocalHostIp'])
				robot.setPort(airsim_settings['Vehicles'][vehicle]['TcpPort'])
				robot.setSITL(airsim_settings['Vehicles'][vehicle]['UseSerial'])
			except (RuntimeError, TypeError, NameError):
				print('Error reading LocalHostIP, TcpPort, or UseSerial in settings.json for robot '+vehicle)	

			if (not robot.sitl):
				print('Starting px4_sitl for '+vehicle)
				robot.setPX4(subprocess.Popen(["make","px4_sitl_default","none_iris"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="../px4/Firmware"))
			transforms = ASP.getTransforms(airsim_settings['Vehicles'][vehicle], robot)

			#add ned transforms
			ned2enu = TransformStamped()
			ned2enu.header.frame_id = robot.name
			ned2enu.header.stamp = rospy.Time.now()
			ned2enu.child_frame_id = robot.name  + "_ned"
			ned2enu.transform.translation.x = 0.0
			ned2enu.transform.translation.y = 0.0
			ned2enu.transform.translation.z = 0.0

			q = quaternion_from_euler(180*(math.pi/180), 0, 90*(math.pi/180))
			ned2enu.transform.rotation.x = q[0]
			ned2enu.transform.rotation.y = q[1]
			ned2enu.transform.rotation.z = q[2]
			ned2enu.transform.rotation.w = q[3]
			transforms.append(ned2enu)
			#add orb transforms
			orb2enu = TransformStamped()
			orb2enu.header.frame_id = robot.name
			orb2enu.header.stamp = rospy.Time.now()
			orb2enu.child_frame_id = robot.name  + "_orb"
			orb2enu.transform.translation.x = 0.0
			orb2enu.transform.translation.y = 0.0
			orb2enu.transform.translation.z = 0.0

			q = quaternion_from_euler(0, 0, 90*(math.pi/180))
			orb2enu.transform.rotation.x = q[0]
			orb2enu.transform.rotation.y = q[1]
			orb2enu.transform.rotation.z = q[2]
			orb2enu.transform.rotation.w = q[3]
			transforms.append(orb2enu)
	
			static_br.sendTransform(transforms)
			ASP.addRobot(robot)

		#Begin Loop
		rate = rospy.Rate(30)  # 30hz
		while not rospy.is_shutdown():
			#TODO: multithreading
			ASP.updateTime()
			for robot in ASP.robots:
				transforms = ASP.updateData(robot)		
			rate.sleep()

class Robot:
	def __init__(self, name):
		self.name = name
		self.address = '0.0.0.0'
		self.port = '0'
		self.sitl = False
		self.px4 = []
		self.MAVROS = []
		self.sensors = {}
		self.cameras = {}
		self.publishers = {}
		self.publishers['truepose'] = rospy.Publisher('airsim/' + self.name + "/truepose", PoseStamped, queue_size=10)	

	def setAddress(self, address):
		self.address = address

	def setPort(self, port):
		self.port = port

	def setSITL(self, enable):
		if (enable == 'True' or enable == 'true'):
			self.sitl = True

	def setPX4(self, subprocess):
		self.px4 = subprocess

	def setMAVROS(self, ros_process):
		self.MAVROS = ros_process

	def addCameras(self, cameras):
		camera_dict = {}
		for camera in cameras:
			camera_params = cameras[camera]['CaptureSettings'][0]
			if ('PublishToRos' in camera_params and camera_params['PublishToRos'] == 1):			
				camera_dict[camera] = cameras[camera]
				self.publishers[camera] = rospy.Publisher('airsim/' + self.name + "/" + camera + "/image_raw", Image, queue_size=30)
				self.publishers[camera + "_info"] = rospy.Publisher('airsim/' + self.name + "/" + camera, CameraInfo, queue_size=30)
		self.cameras = camera_dict

class AirSimPublisher:
	def __init__(self):
		self.robots = []
		self.client = airsim.MultirotorClient()
		self.time = rospy.Time.now()
		self.bridge = CvBridge()
		self.tfBuffer = tf2_ros.Buffer()
    		listener = tf2_ros.TransformListener(self.tfBuffer)

	def updateTime(self):
		self.time = rospy.Time.now()

	def addRobot(self, robot):
		self.robots.append(robot)

	def getCameraInfoMsg(self,camera,robot):
		cam_params = robot.cameras[camera]['CaptureSettings'][0]
		height = cam_params['Height']
		width = cam_params['Width']
		fx = abs((width/2)/math.tan(cam_params['FOV_Degrees']/2*math.pi/180))
		fy = fx		
		#fy = abs((height/2)/math.tan(cam_params['FOV_Degrees']/2*math.pi/180))
		cx = width/2
		cy = height/2		
		cam_msg = CameraInfo()
		cam_msg.header.stamp = self.time
		cam_msg.header.frame_id = str(camera)
		cam_msg.height = height
		cam_msg.width = width
		cam_msg.distortion_model = "plumb_bob"
		cam_msg.D = (0, 0, 0, 0, 0)
		cam_msg.K = (fx, 0, cx, 0, fy, cy, 0, 0, 1)
		return cam_msg

	def getImageMsg(self,image,camera,robot):
		cam_params = robot.cameras[camera]['CaptureSettings'][0]
		height = cam_params['Height']
		width = cam_params['Width']

		img_msg = Image()
		img_msg.header.stamp = self.time
		img_msg.header.frame_id = str(camera)
		img_msg.encoding = "bgr8"
		img_msg.height = height
		img_msg.width = width
		img_msg.data = image
		img_msg.is_bigendian = 0
		img_msg.step = img_msg.width * 3
		return img_msg

	def getPoseMsg(self,robot):
		pose_data = self.client.simGetVehiclePose(robot.name)
		
		ned_pose = PoseStamped()
		ned_pose.header.stamp = self.time
		ned_pose.header.frame_id = robot.name + "_ned"
		ned_pose.pose.position = Point(pose_data.position.x_val, pose_data.position.y_val, pose_data.position.z_val)
		ned_pose.pose.orientation = Quaternion(pose_data.orientation.x_val, pose_data.orientation.y_val, pose_data.orientation.z_val, pose_data.orientation.w_val)
		
		#pose_msg = PoseStamped()
		#pose_msg = tf2_geometry_msgs.do_transform_pose(ned_pose, t)
		
		return ned_pose

	def getTransforms(self, settings, robot):
		#https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md#3.1
		cameras = settings['Cameras']
		transforms = []
		for camera in cameras:
			#gimbal cameras are not going to be static transforms
			if not ('Gimbal' in cameras[camera]):
				transforms.append(self.staticTransform(cameras[camera], str(camera), robot.name))
		robot_2_map = TransformStamped()
		robot_2_map.header.stamp = self.time
		robot_2_map.header.frame_id = 'odom'
		robot_2_map.child_frame_id = robot.name
		robot_2_map.transform.rotation = Quaternion(0,0,0,1)
		transforms.append(robot_2_map)
		map_2_odom = TransformStamped()
		map_2_odom.header.stamp = self.time
		map_2_odom.header.frame_id = 'map'
		map_2_odom.child_frame_id = 'odom'
		map_2_odom.transform.rotation = Quaternion(0,0,0,1)
		transforms.append(map_2_odom)
		return transforms

	def staticTransform(self, sensor, child, parent):
		x = sensor['X'] if ('X' in sensor) else 0
		y = sensor['Y'] if ('Y' in sensor) else 0
		z = sensor['Z'] if ('Z' in sensor) else 0
		roll = -sensor['Roll']*(math.pi/180) if ('Roll' in sensor) else 0
		pitch = -sensor['Pitch']*(math.pi/180) if ('Pitch' in sensor) else 0
		yaw = -sensor['Yaw']*(math.pi/180) if ('Yaw' in sensor) else 0

		transform = TransformStamped()
		transform.header.stamp = self.time
		transform.header.frame_id = 'base_link_orb'
		transform.child_frame_id = 'camera_link'
		print(parent,child)
		q = quaternion_from_euler(roll, pitch, yaw)
		transform.transform.translation = Point(x,y,z)
		transform.transform.rotation.x = q[0]
		transform.transform.rotation.y = q[1]
		transform.transform.rotation.z = q[2]
		transform.transform.rotation.w = q[3]
		return transform

	def updateData(self, robot):
		img_pub = []
		cam_pub = []
		img_requests = []		
		transforms = []
		for camera in robot.cameras:
			img_pub.append(robot.publishers[camera])
			cam_pub = robot.publishers[camera + "_info"]
			cam_pub.publish(self.getCameraInfoMsg(camera,robot))	
			img_type = robot.cameras[camera]['CaptureSettings'][0]['ImageType']
			img_requests.append(airsim.ImageRequest(camera, img_type, False, False))
			#TODO publish gimbal transform			
			#if ('Gimbal' in robot.cameras[camera]):			
			
		#get images
		responses = self.client.simGetImages(img_requests, robot.name)
		if len(responses) != len(robot.cameras):
			print("Error receiving images from AirSim")
		else:
			for i in range(len(responses)):
				img_pub[i].publish(self.getImageMsg(responses[i].image_data_uint8,robot.cameras.keys()[i],robot))
		#update true pose
		pub = robot.publishers['truepose']
		pub.publish(self.getPoseMsg(robot))



