from AirSimClient import *
import sys
import time

print("connecting to drone...")
client = MultirotorClient()

client.waitForReadyState()

print("arming the drone...")
client.armDisarm(True)

print("taking off...")
client.takeoff()

client.enableApiControl(True)
# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing max_wait_seconds=0.
print("client.moveOnPath to fly fast path along the streets")
try:
    result = client.moveOnPath([Vector3r(0,-253,z),Vector3r(125,-253,z),Vector3r(125,0,z),Vector3r(0,0,z),Vector3r(0,0,-20)], 
                           15, 65, 
                           DrivetrainType.ForwardOnly, YawMode(False,0), 20, 1)
except:
    errorType, value, traceback = sys.exc_info()
    print("moveOnPath threw exception: " + str(value))
    pass

print("landing...")
client.land()

print("disarming.")
client.armDisarm(False)