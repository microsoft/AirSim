from PythonClient import *
import sys
import time

client = AirSimClient('127.0.0.1')
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoff()

# AirSim uses NED coordinates so negative axis is up.
# z of -5 is 5 meters above the original launch point.
z = -5

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing max_wait_seconds=0.
print("client.moveOnPath to fly fast path along the streets")
result = client.moveOnPath([Vector3r(0,-253,z),Vector3r(125,-253,z),Vector3r(125,0,z),Vector3r(0,0,z)], 
                           15, 60, 
                           DrivetrainType.ForwardOnly, YawMode(False,0), 20, 1)
