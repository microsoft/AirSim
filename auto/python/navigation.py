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
		goal.height = 22
	
		print('Actionlib started')

		takeoff_client.send_goal(goal)
		takeoff_client.wait_for_result()

#1: Hold	Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)	min:0	s
#2: Accept Radius	Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)	min:0	m
#3: Pass Radius	0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit.
#4: Yaw	Desired yaw angle (deg) at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
#5: Latitude	Latitude		
#6: Longitude	Longitude		
#7: Altitude	Altitude

		wps = []
		wp1 = Waypoint()
		wp2 = Waypoint()
		wp3 = Waypoint()
		wp4 = Waypoint()
		wp5 = Waypoint()
		wp6 = Waypoint()
		wp7 = Waypoint()
		wp8 = Waypoint()
		wp9 = Waypoint()
		wp10 = Waypoint()
		wp11 = Waypoint()
		wp12 = Waypoint()

		wp1.command = 16
		#wp1.param4 = np.nan
		wp1.x_lat = 40.00482126
		wp1.y_long = 100.0100724
		wp1.z_alt = 22
		wp1.autocontinue = True

		wp2.command = 16
		#wp3.param4 = np.nan
		wp2.x_lat = 40.00422467
		wp2.y_long = 100.0122255
		wp2.z_alt = 22
		wp2.autocontinue = True


		wp3.command = 16
		#wp3.param4 = np.nan
		wp3.x_lat = 40.00443838
		wp3.y_long = 100.0123931
		wp3.z_alt = 22
		wp3.autocontinue = True

		wp4.command = 16
		#wp4.param4 = np.nan
		wp4.x_lat = 40.00515832
		wp4.y_long = 100.0103007
		wp4.z_alt = 22
		wp4.autocontinue = True

		wp5.command = 16
		#wp5.param4 = np.nan
		wp5.x_lat = 40.00549537
		wp5.y_long = 100.010529
		wp5.z_alt = 22
		wp5.autocontinue = True

		wp6.command = 16
		#wp6.param4 = np.nan
		wp6.x_lat = 40.0046521
		wp6.y_long = 100.0125607
		wp6.z_alt = 22
		wp6.autocontinue = True

		wp7.command = 16
		#wp7.param4 = np.nan
		wp7.x_lat = 40.00489078
		wp7.y_long = 100.0127564
		wp7.z_alt = 22
		wp7.autocontinue = True

		wp8.command = 16
		#wp8.param4 = np.nan
		wp8.x_lat = 40.00573685
		wp8.y_long = 100.0106538
		wp8.z_alt = 22
		wp8.autocontinue = True

		wp9.command = 16
		#wp9.param4 = np.nan
		wp9.x_lat = 40.00597832
		wp9.y_long = 100.0107787
		wp9.z_alt = 22
		wp9.autocontinue = True

		wp10.command = 16
		#wp10.param4 = np.nan
		wp10.x_lat = 40.00512946
		wp10.y_long = 100.0129521
		wp10.z_alt = 22
		wp10.autocontinue = True

		wp11.command = 16
		#wp11.param4 = np.nan
		wp11.x_lat = 40.00536815
		wp11.y_long = 100.0131477
		wp11.z_alt = 22
		wp11.autocontinue = True

		wp12.command = 16
		#wp12.param4 = np.nan
		wp12.x_lat = 40.00621979
		wp12.y_long = 100.0109035
		wp12.z_alt = 22
		wp12.autocontinue = True

		goal = WaypointsGoal()
		goal.waypoints.append(wp1)
		goal.waypoints.append(wp2)
		goal.waypoints.append(wp3)
		goal.waypoints.append(wp4)
		goal.waypoints.append(wp5)
		goal.waypoints.append(wp6)
		goal.waypoints.append(wp7)
		goal.waypoints.append(wp8)
		goal.waypoints.append(wp9)
		goal.waypoints.append(wp10)
		goal.waypoints.append(wp11)
		goal.waypoints.append(wp12)
		print(goal)
		waypoints_client.send_goal(goal)
		waypoints_client.wait_for_result(rospy.Duration.from_sec(800.0))

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




