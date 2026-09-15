#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import TakeoffRequest,Takeoff

msg = """
Control the RoboMaster TT (Tello Talent) through Gamepad
---------------------------
Moving around:
Hold LB button to enable manual control
LEFT HAND AXIS:             RIGHT HAND AXIS:

            +throttle                       +Pitch
                                    
  + yaw                 -yaw       +roll                 -roll
                                    
            -throttle                       -Pitch
  
Buttons:
up : to takeoff
down: to land
Y: to flip forward

CTRL-C to quit
"""

class UAVJoyTeleop(object):
    """UAV joy teleop node."""

    def __init__(self):
        print(msg)
        rospy.loginfo("Joy Teleop Initializing...")
        self.curr_vehicle_name = "Drone_1"
        self._twist = self._zero_twist = VelCmd()
        self._takeoff_signal = TakeoffRequest()
        self._takeoff_signal.waitOnLastTask = True
        self._deadman_pressed = False
        self._zero_twist_published = False

        self._cmd_vel_pub = rospy.Publisher("/airsim_node/" + self.curr_vehicle_name + "/vel_cmd_body_frame", VelCmd, queue_size=5)
        rospy.wait_for_service("/airsim_node/" + self.curr_vehicle_name + "/takeoff")
        #self._takeoff_client = rospy.ServiceProxy("/airsim_node/" + self.curr_vehicle_name + "/takeoff", Takeoff)
        #self._land_pub = rospy.ServiceProxy("/airsim_node/" + self.curr_vehicle_name + "/land", Empty)
        #self._flip_pub = rospy.Publisher('flip', Empty, queue_size=1)
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self._timer = rospy.Timer(rospy.Duration(0.01), self.joystick_controller)

        _joy_mode = rospy.get_param("~joy_mode", "D").lower()        
        if _joy_mode == "d":
            self._axis_yaw = 0
            self._axis_throttle = 1
            self._axis_roll = 3
            self._axis_pitch = 4
            self._axis_takeoff_land = 2
        
        #self._takeoff_client.call(self._takeoff_signal)
        self._linear_scale = rospy.get_param("~linear_scale", 2)
        self._angular_scale = rospy.get_param("~angular_scale", 1.5)

        rospy.loginfo("RMTT Joy Teleop Initializing...Done")

    def joy_callback(self, joy):
        print("joy.axes[self._axis_pitch]",joy.axes[self._axis_pitch])
        print("joy.axes[self._axis_roll]",joy.axes[self._axis_roll]) 
        print("joy.axes[self._axis_throttle]",joy.axes[self._axis_throttle]) 
        print("joy.axes[self._axis_yaw]",joy.axes[self._axis_yaw])        
        self._twist.twist.linear.x = joy.axes[self._axis_pitch]**3.0*self._linear_scale
        self._twist.twist.linear.y = -joy.axes[self._axis_roll]**3.0*self._linear_scale
        self._twist.twist.linear.z = -joy.axes[self._axis_throttle]**3.0*self._linear_scale
        self._twist.twist.angular.x = 0
        self._twist.twist.angular.y = 0
        self._twist.twist.angular.z = -joy.axes[self._axis_yaw]**3.0*self._angular_scale


    def joystick_controller(self, *args):
        self._cmd_vel_pub.publish(self._twist)

if __name__ == '__main__':
    try:
        rospy.init_node('uav_joy', anonymous=True)
    except rospy.ROSInterruptException:
        rospy.logwarn("UAV Joy Teleop node init failed...")
    else:
        UAVJoyTeleop()
        rospy.spin()