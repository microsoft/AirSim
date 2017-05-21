from PythonClient import *
import sys
import time

client = AirSimClient('127.0.0.1')

# AirSim uses NED coordinates so negative axis is up.
# z of -5 is 5 meters above the original launch point.
z = -5

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing max_wait_seconds=0.
print("client.moveOnPath to fly fast path along the streets")
result = client.moveOnPath([(0,-253,z),(125,-253,z),(125,0,z),(0,0,z)], 15, 0, DrivetrainType.ForwardOnly, YawMode(False,0), 20, 1)
