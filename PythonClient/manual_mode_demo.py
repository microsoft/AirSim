"""
For connecting to the AirSim drone environment and testing API functionality
"""

import os
import tempfile
import pprint

from AirSimClient import *


# connect to the AirSim simulator
client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

client.moveByManual(vx_max = 1E6, vy_max = 1E6, z_min = -1E6, duration = 1E10)
AirSimClientBase.wait_key('Manual mode is setup. Press any key to send RC data to takeoff')

client.setRCData(rcdata = RCData(pitch = 0.0, throttle = 1.0, is_initialized = True, is_valid = True))

AirSimClientBase.wait_key('Set Yaw and pitch to 0.5')

client.setRCData(rcdata = RCData(roll = 0.5, throttle = 1.0, yaw = 0.5, is_initialized = True, is_valid = True))
