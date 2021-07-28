import setup_path
import airsim

import sys
import time

print("""This script is designed to fly on the streets of the Neighborhood environment
and assumes the unreal position of the drone is [160, -1500, 120].""")

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

print("arming the drone...")
client.armDisarm(True)

state = client.getMultirotorState()
if state.landed_state == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    client.hoverAsync().join()

time.sleep(1)

state = client.getMultirotorState()
if state.landed_state == airsim.LandedState.Landed:
    print("take off failed...")
    sys.exit(1)

# AirSim uses NED coordinates so negative axis is up.
# z of -5 is 5 meters above the original launch point.
z = -5
print("make sure we are hovering at {} meters...".format(-z))
client.moveToZAsync(z, 1).join()

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing timeout_sec=0.

print("flying on path...")
result = client.moveOnPathAsync([airsim.Vector3r(125,0,z),
                                airsim.Vector3r(125,-130,z),
                                airsim.Vector3r(0,-130,z),
                                airsim.Vector3r(0,0,z)],
                        12, 120,
                        airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()

# drone will over-shoot so we bring it back to the start point before landing.
client.moveToPositionAsync(0,0,z,1).join()
print("landing...")
client.landAsync().join()
print("disarming...")
client.armDisarm(False)
client.enableApiControl(False)
print("done.")
