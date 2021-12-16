#!/usr/bin/env python

#capture joystick events using ROS and convert to AirSim Car API commands
#to enable:
# rosrun joy joy_node

import rospy
import threading
import sensor_msgs
import sensor_msgs.msg
import airsim_ros_pkgs as air
import airsim_ros_pkgs.msg

class CarCommandTranslator(object):
    def __init__(self):
        self.lock = threading.Lock()

        self.last_forward_btn = 0
        self.last_reverse_btn = 0
        self.last_neutral_btn = 0
        self.last_park_btn = 0
        self.last_shift_down_btn = 0
        self.last_shift_up_btn = 0
        self.parked = True
        self.last_gear = 0
        self.shift_mode_manual = True

        update_rate_hz = rospy.get_param('~update_rate_hz', 20.0)
        self.max_curvature = rospy.get_param('~max_curvature', 0.75)
        self.steer_sign = rospy.get_param('~steer_sign', -1)
        self.throttle_brake_sign = rospy.get_param('~throttle_brake_sign', 1)
        self.auto_gear_max = rospy.get_param('~auto_gear_max', 5)
        self.manual_transmission = rospy.get_param('~manual_transmission', True)
        self.forward_btn_index = rospy.get_param('~forward_button_index', 0)
        self.reverse_btn_index = rospy.get_param('~reverse_button_index', 1)
        # Below was an earlier typo, written like this for compatibility
        self.neutral_btn_index = rospy.get_param('~neutral_button_index', rospy.get_param('~nuetral_button_index', 2))
        self.park_btn_index = rospy.get_param('~park_button_index', 3)
        self.shift_down_btn_index = rospy.get_param('~shift_down_index', 4)
        self.shift_up_btn_index = rospy.get_param('~shift_up_index', 5)
        car_control_topic = rospy.get_param('~car_control_topic', '/airsim_node/drone_1/car_cmd')

        self.joy_msg = None

        self.joy_sub = rospy.Subscriber(
            'joy',
            sensor_msgs.msg.Joy,
            self.handle_joy)

        self.command_pub = rospy.Publisher(
            car_control_topic,
            air.msg.CarControls,
            queue_size=0
        )

        self.update_time = rospy.Timer(
            rospy.Duration(1.0/update_rate_hz),
            self.handle_update_timer
        )

    def handle_joy(self, msg):
        with self.lock:
            self.joy_msg = msg

    def handle_update_timer(self, ignored):
        joy = None
        with self.lock:
            joy = self.joy_msg

        if joy is None:
            return

        controls = airsim_ros_pkgs.msg.CarControls()

        controls.steering = self.steer_sign * self.max_curvature * joy.axes[2]
        u = joy.axes[1] * self.throttle_brake_sign
        if u > 0.0:
            controls.throttle = abs(u)
            controls.brake = 0.0
        else:
            controls.throttle = 0.0
            controls.brake = abs(u)      

        forward_btn    = joy.buttons[self.forward_btn_index]
        reverse_btn    = joy.buttons[self.reverse_btn_index]
        neutral_btn    = joy.buttons[self.neutral_btn_index]
        park_btn       = joy.buttons[self.park_btn_index]
        shift_up_btn   = joy.buttons[self.shift_up_btn_index]
        shift_down_btn = joy.buttons[self.shift_down_btn_index]
        

        # gearing: -1 reverse, 0 N, >= 1 drive
        controls.manual = True #set to False for automatic transmission along with manual_gear > 1
        if not self.last_neutral_btn and neutral_btn:
            self.last_gear = 0
            self.parked = False
            controls.manual = True
        elif not self.last_forward_btn and forward_btn:
            if self.manual_transmission:
                self.last_gear = 1
                self.shift_mode_manual = True
            else:
                self.shift_mode_manual = False
                self.last_gear = self.auto_gear_max

            self.parked = False
        elif not self.last_reverse_btn and reverse_btn:
            self.last_gear = -1
            self.parked = False
            self.shift_mode_manual = True
        elif not self.last_park_btn and park_btn:
            self.parked = True
        elif not self.last_shift_down_btn and shift_down_btn and self.last_gear > 1 and self.manual_transmission:
            self.last_gear-=1
            self.parked = False
            self.shift_mode_manual = True
        elif not self.last_shift_up_btn and shift_up_btn and self.last_gear >= 1 and self.manual_transmission:
            self.last_gear+=1
            self.parked = False
            self.shift_mode_manual = True

        if self.parked:
            self.last_gear = 0
            self.shift_mode_manual = True
            controls.handbrake = True
        else:
            controls.handbrake = False

        controls.manual_gear = self.last_gear
        controls.manual = self.shift_mode_manual
        
        now = rospy.Time.now()
        controls.header.stamp = now
        controls.gear_immediate = True

        self.last_neutral_btn = neutral_btn
        self.last_forward_btn = forward_btn
        self.last_reverse_btn = reverse_btn
        self.last_park_btn = park_btn
        self.last_shift_down_btn = shift_down_btn
        self.last_shift_up_btn = shift_up_btn

        self.command_pub.publish(controls)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('car_joy')
    node = CarCommandTranslator()
    node.run()
