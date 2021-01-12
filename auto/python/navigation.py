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
from mavros import mavlink
from mavros import action_server
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached, GlobalPositionTarget, State, TakeoffAction, TakeoffGoal, LandAction, LandGoal, WaypointsAction, WaypointsGoal, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear, CommandHome
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from threading import Thread

# Brings in the SimpleActionClient
import actionlib

class offboard():

	def state_callback(self, data):
		self.state = data

	def wp_reached_callback(self, data):
		self.wp_reached = data

	def home_pos_callback(self, data):
		self.home_pos = data
		#print(self.home_pos.geo)

	def global_pos_callback(self, data):
		self.global_pos = data

	def __init__(self):

   		rospy.init_node('guidance_node', anonymous=True)

		state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
		self.state = State
		#PUBLISHERS
		local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		#global_pos_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
		local_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_pos_callback)
		home_pos_sub = rospy.Subscriber('/mavros/home_position/home', HomePosition, self.home_pos_callback)

		#ACTIONS
		#init actionlib servers
		server = Thread(target=action_server.ActionServer)
		server.setDaemon(True)
		server.start()
		takeoff_client = actionlib.SimpleActionClient('takeoff', TakeoffAction)
		land_client = actionlib.SimpleActionClient('land', LandAction)
		waypoints_client = actionlib.SimpleActionClient('waypoints', WaypointsAction)

		# need to simulate heartbeat to prevent datalink loss detection
		hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
		hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
		hb_ros_msg = mavlink.convert_to_rosmsg(hb_mav_msg)
		hb_thread = Thread(target=self.send_heartbeat, args=(hb_ros_msg))
		hb_thread.setDaemon(True)

		#PREFLIGHT CHECK

		rate = rospy.Rate(30)
		while (not self.state.connected):
			print('Waiting on Connection')
			rate.sleep()
		print('Connected')

		time.sleep(5)

		goal = TakeoffGoal()
		goal.height = 10
	
		print('Actionlib started')

		takeoff_client.send_goal(goal)
		takeoff_client.wait_for_result()

		wps = []
		wp1 = Waypoint()
		wp2 = Waypoint()
		wp3 = Waypoint()
		wp1.command = 16
		wp1.x_lat = self.home_pos.geo.latitude - 0.00050
		wp1.y_long = self.home_pos.geo.longitude
		wp1.z_alt = 10
		wp1.autocontinue = True

		wp2.command = 16
		wp2.x_lat = self.home_pos.geo.latitude - 0.00050
		wp2.y_long = self.home_pos.geo.longitude - 0.00050
		wp2.z_alt = 10
		wp2.autocontinue = True


		wp3.command = 16
		wp3.x_lat = self.home_pos.geo.latitude
		wp3.y_long = self.home_pos.geo.longitude
		wp3.z_alt = 10
		wp3.autocontinue = True

		goal = WaypointsGoal()
		goal.waypoints.append(wp1)
		goal.waypoints.append(wp2)
		goal.waypoints.append(wp3)
		print(goal)
		waypoints_client.send_goal(goal)
		waypoints_client.wait_for_result(rospy.Duration.from_sec(45.0))

		time.sleep(5)

		goal = LandGoal()
		goal.x_lat = self.home_pos.geo.latitude
		goal.y_long = self.home_pos.geo.longitude
		goal.z_alt = 0.0

		print('Actionlib started')

		land_client.send_goal(goal)
		land_client.wait_for_result(rospy.Duration.from_sec(30.0))
		sys.exit()
		

	# Heartbeat must be sent to px4 at 2Hz or else auto disconnect
	def send_heartbeat(self, hb_ros_msg):
		rate = rospy.Rate(2)  # Hz
		while not rospy.is_shutdown():
			self.mavlink_pub.publish(hb_ros_msg)
			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass


if __name__ == '__main__':
	offboard()




