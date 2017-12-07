from AirSimClient import *
import sys
import time

print("connecting to drone...")
client = MultirotorClient()

client.waitForReadyState()

print("arming the drone...")
client.armDisarm(True)

# note: here we need to wait until the PX4 has estabilished the home position, which it
# does after it is done with GPS EKF fusion (checking GPS signal).  This can take up to
# a minute or so...if you try and takeoff before that then the takeoff is rejected.

print("attempting takeoff")
while True:
    try:
        client.takeoff()
        time.sleep(2) # give it a couple seconds to stabilize
        break
    except:
        print("takeoff failed, trying again in 5 seconds...")
        time.sleep(5)

client.enableApiControl(True)
# AirSim uses NED coordinates so negative axis is up.
# z of -5 is 5 meters above the original launch point.
z = -5

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing max_wait_seconds=0.
print("client.moveOnPath to fly fast path along the streets")
try:
    result = client.moveOnPath([Vector3r(0,-253,z),Vector3r(125,-253,z),Vector3r(125,0,z),Vector3r(0,0,z),Vector3r(0,0,-20)], 
                           15, 65, 
                           DrivetrainType.ForwardOnly, YawMode(False,0), 20, 1)
except:
    pass

print("landing...")
client.land()

print("disarming.")
client.armDisarm(False)