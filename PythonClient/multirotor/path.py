import setup_path
import airsim

import sys
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

print("arming the drone...")
client.armDisarm(True)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    client.hoverAsync().join()

time.sleep(1)
landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("take off failed...")
    sys.exit(1)

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7
print("make sure we are hovering at 7 meters...")
client.moveToZAsync(z, 1).join()

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing timeout_sec=0.

print("flying on path...")
result = client.moveOnPathAsync([airsim.Vector3r(0,-253,z),airsim.Vector3r(125,-253,z),airsim.Vector3r(125,5,z),airsim.Vector3r(5,0,z)],
                        12, 120,
                        airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()
client.hoverAsync().join()
client.moveToPositionAsync(0,0,z,1).join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
