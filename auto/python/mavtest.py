import rospy
import glob
import json
import math
import os
import px4tools
import sys
from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached, GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from threading import Thread

class mavrostest():

	def state_callback(self, data):
		self.state = data

	def wp_reached_callback(self, data):
		self.wp_reached = data

	def global_pos_callback(self, data):
		self.global_pos = data

	def __init__(self):

   		rospy.init_node('test_node', anonymous=True)
		self.state = State()
		self.wp_reached = 0
		self.global_pos = NavSatFix()
		
		#SUBSCRIBERS
		state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
		#global_pos_sub = rospy.Subscriber('/mavros/global_position/global', State, self.state_callback)
		local_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_pos_callback)
		wp_reached_sub = rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.state_callback)

		#PUBLISHERS
		local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		#global_pos_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)

		#SERVICES
		arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		takeoff_client = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
		land_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
		mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
		mission_push_client = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
		mission_clear_client = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)

		rate = rospy.Rate(20)

		while (not self.state.connected):
			print('Waiting on Connection')
			rate.sleep()
		print('Connected')

		# need to simulate heartbeat to prevent datalink loss detection
		hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
		hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
		hb_ros_msg = mavlink.convert_to_rosmsg(hb_mav_msg)
		hb_thread = Thread(target=self.send_heartbeat, args=(hb_ros_msg))

		last_request = rospy.Time.now()
		# Disarm
		ack = False
		while (not ack):
			try:
				ack = arm_client(False).success
			except rospy.ServiceException as e:
				print("Disarming Failed: %s" %e)
			rate.sleep()
		print('Disarmed')

		# Set Mode
		mode = "AUTO.LOITER"
		ack = False
		while (not ack):
			try:
				ack = mode_client(0, "AUTO.LOITER").mode_sent  # 0 is custom mode
			except rospy.ServiceException as e:
				print("Mode Change Failed: %s" %e)
			rate.sleep()
		print('Mode set to ', mode)

		# Arm
		ack = False
		while (not ack):
			try:
				ack = arm_client(True).success
			except rospy.ServiceException as e:
				print("Arming Failed: %s" %e)
			rate.sleep()
		print('Armed')

		#Clear any old missions
		ack = False
		while (not ack):
			try:
				ack = mission_clear_client().success
			except rospy.ServiceException as e:
				print("Mission Clear Failed: %s" %e)
			rate.sleep()
		print('old missions cleared')

		#Create and execute Mission

		home_lat = self.global_pos.latitude
		home_long = self.global_pos.longitude

		waypoints = []
		takeoff = Waypoint()
		takeoff.frame = 3
		takeoff.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
		takeoff.is_current = True
		takeoff.autocontinue = True
		takeoff.param1 = 0.0
		takeoff.param2 = 0.0
		takeoff.param3 = 0.3
		takeoff.param4 = 0.0
		takeoff.x_lat = home_lat
		takeoff.y_long = home_long
		takeoff.z_alt = 8.0

		wp1 = Waypoint()
		wp1.frame = 3
		wp1.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
		wp1.is_current = True
		wp1.autocontinue = True
		wp1.param1 = 0.0
		wp1.param2 = 0.0
		wp1.param3 = 0.3
		wp1.param4 = 0.0
		wp1.x_lat = home_lat + 0.00005
		wp1.y_long = home_long
		wp1.z_alt = 8.0

		waypoints.append(wp1)

		rtl = Waypoint()
		rtl.frame = 3
		rtl.command = 20
		rtl.is_current = True
		rtl.autocontinue = True
		rtl.param1 = 0.0
		rtl.param2 = 0.0
		rtl.param3 = 0.0
		rtl.param4 = 0.0
		rtl.x_lat = 0.0
		rtl.y_long = 0.0
		rtl.z_alt = 0.0
		waypoints.append(rtl)

	
		ack = False
		while (not ack):	
			try:
				ack = mission_push_client(start_index=0, waypoints=waypoints).success
			except rospy.ServiceException as e:
				print("Mission Push Failed: %s" %e)
			rate.sleep()

		print('Mission Loaded')

		# Set Mode
		mode = "AUTO.MISSION"
		ack = False
		while (not ack):
			try:
				ack = mode_client(5, mode).mode_sent  # 0 is custom mode
			except rospy.ServiceException as e:
				print("Mode Change Failed: %s" %e)
			rate.sleep()

		print('Beginning Mission')

		while (self.wp_reached != 3):
			rate.sleep()

#		print "\nTaking off"
#		try:
#			response = takeoff_client(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
#			rospy.loginfo(response)
#		except rospy.ServiceException as e:
#			print("Takeoff failed: %s" %e)


	#
	# Helper methods
	#
	def send_heartbeat(self, hb_ros_msg):
		rate = rospy.Rate(2)  # Hz
		while not rospy.is_shutdown():
			self.mavlink_pub.publish(hb_ros_msg)
			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

if __name__ == '__main__':
	mavrostest()

