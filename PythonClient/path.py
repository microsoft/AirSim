from AirSimClient import *
import sys
import time

client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)
client.takeoff()

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing max_wait_seconds=0.
result = client.moveOnPath([Vector3r(0,-253,z),Vector3r(125,-253,z),Vector3r(125,0,z),Vector3r(0,0,z),Vector3r(0,0,-20)], 
                        12, 120, 
                        DrivetrainType.ForwardOnly, YawMode(False,0), 20, 1)
client.moveToPosition(0,0,z,1)
client.land()
client.armDisarm(False)
client.enableApiControl(False)
