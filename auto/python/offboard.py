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
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached, GlobalPositionTarget, State, TakeoffAction, TakeoffGoal, LandAction, LandGoal, WaypointsAction, WaypointsGoal
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from threading import Thread

# Brings in the SimpleActionClient
import actionlib

class offboard(MavrosTestCommon):

	def state_callback(self, data):
		self.state = data

	def wp_reached_callback(self, data):
		self.wp_reached = data

	def global_pos_callback(self, data):
		self.global_pos = data

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

   		rospy.init_node('test_node', anonymous=True)

		print('got here')
		
		#SUBSCRIBERS
		state_sub = rospy.Subscriber('mavros/state', State, self.state_callback)
		#global_pos_sub = rospy.Subscriber('/mavros/global_position/global', State, self.state_callback)
		local_pos_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_pos_callback)
		#wp_reached_sub = rospy.Subscriber('/mavros/mission/reached', WaypointReached, self.wp_reached_callback)

		#PUBLISHERS
		local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		#global_pos_pub = rospy.Publisher('mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)

		#SERVICES
		arm_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		#takeoff_client = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
		#land_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
		mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
		#mission_push_client = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
		mission_clear_client = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)

		#ACTIONS
		#init actionlib servers
		server = Thread(target=action_server.ActionServer)
		server.setDaemon(True)
		server.start()
		takeoff_client = actionlib.SimpleActionClient('takeoff', TakeoffAction)
		land_client = actionlib.SimpleActionClient('land', LandAction)
		waypoints_client = actionlib.SimpleActionClient('waypoints', WaypointsAction)

		self.state = State()
		self.wp_reached = 0
		self.global_pos = NavSatFix()

		rate = rospy.Rate(30)
		while (not self.state.connected):
			print('Waiting on Connection')
			rate.sleep()
		print('Connected')

		# need to simulate heartbeat to prevent datalink loss detection
		hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
		hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
		hb_ros_msg = mavlink.convert_to_rosmsg(hb_mav_msg)
		hb_thread = Thread(target=self.send_heartbeat, args=(hb_ros_msg))
		hb_thread.setDaemon(True)

		#PREFLIGHT CHECK

		time.sleep(5)

		#Clear any old missions
		ack = False
		while (not ack):
			try:
				ack = mission_clear_client().success
			except rospy.ServiceException as e:
				print("Mission Clear Failed: %s" %e)
			rate.sleep()
		print('old missions cleared')

		time.sleep(5)

		# Arm
		self.set_arm(True, 5)

		goal = TakeoffGoal()
		goal.height = 10
	
		print('Actionlib started')

		takeoff_client.send_goal(goal)
		takeoff_client.wait_for_result()

		wp1 = Waypoint()
		wp2 = Waypoint()
		wp1.x_lat = self.global_pos.latitude + 0.00110
		wp1.y_long = self.global_pos.longitude + 0.00110
		wp1.z_alt = 10

		wp2.x_lat = self.global_pos.latitude
		wp2.y_long = self.global_pos.longitude
		wp2.z_alt = 10

		goal = WaypointsGoal()
		goal.waypoints = [wp1, wp2]
		#waypoints_client.send_goal(goal)
		#waypoints_client.wait_for_result(rospy.Duration.from_sec(45.0))

		goal = LandGoal()
		goal.height = 0.0
	
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




