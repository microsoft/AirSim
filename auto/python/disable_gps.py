import rospy
import px4tools

from mavros import mavlink
from mavros_msgs.msg import Mavlink, Waypoint, WaypointReached, GlobalPositionTarget, State, TakeoffAction, TakeoffGoal, LandAction, LandGoal, WaypointsAction, WaypointsGoal, HomePosition, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear, CommandHome, ParamSet


rospy.init_node('disable_gps', anonymous=True)

rospy.wait_for_service('/mavros/param/set')
print('connected to ParamSet Service')
param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)

param = ParamValue()
param_set


rospy.spin()


