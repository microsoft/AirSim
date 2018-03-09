"""
For connecting to the AirSim drone environment and testing API functionality
"""

import os
import tempfile
import pprint
import time
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

t1 = time.time()
while(True):
    client.setRCData(rcdata = RCData(pitch = 0.0, throttle = 1.0, is_initialized = True, is_valid = True))
    #time.sleep()
    #client.rotateByYawRate(0.5, 0.1)
    if time.time() - t1 > 3:
        break

AirSimClientBase.wait_key('Set Yaw and pitch to 0.5')
# t1 = time.time()
# while(True):
#     client.setRCData(rcdata = RCData(roll = 0, throttle = 1.0, yaw = 0.5, is_initialized = True, is_valid = True))
#     if time.time() - t1 > 3:
#         break
