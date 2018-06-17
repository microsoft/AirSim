"""
For connecting to the AirSim drone environment and testing API functionality
"""
import setup_path 
import airsim

import os
import tempfile
import pprint

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

client.moveByManualAsync(vx_max = 1E6, vy_max = 1E6, z_min = -1E6, duration = 1E10)
airsim.wait_key('Manual mode is setup. Press any key to send RC data to takeoff')

client.moveByRC(rcdata = airsim.RCData(pitch = 0.0, throttle = 1.0, is_initialized = True, is_valid = True))

airsim.wait_key('Set Yaw and pitch to 0.5')

client.moveByRC(rcdata = airsim.RCData(roll = 0.5, throttle = 1.0, yaw = 0.5, is_initialized = True, is_valid = True))
