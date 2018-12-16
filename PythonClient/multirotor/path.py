
# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo
# this path is calculated to work nicely with the Neighborhood Map, assuming the 
# start location is set to unreal map coordinates (310, 11200, 121)
import setup_path 
import airsim

import sys
import time

def print_position(client):    
    state = client.getMultirotorState()
    gps = state.gps_location
    print("gps location lat={}, lon={}, alt={}".format(gps.latitude, gps.longitude, gps.altitude))
    pos = state.kinematics_estimated.position
    print("local position x={}, y={}, z={}".format(pos.x_val, pos.y_val, pos.z_val))


client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print_position(client)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    client.hoverAsync().join()


if client.getMultirotorState().landed_state == airsim.LandedState.Landed:
    print("take off failed, please check message log")
    sys.exit(1)


# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7

# make sure we are at the start location (previous flight might have missed the landing spot by a bit.)
client.moveToPositionAsync(0,0,z,1).join()

path = [(0,-254, z), (125,-254,z), (125,0,z), (0,0,z)]
print("flying path {}".format(path))

# this method is async, but we are calling .join() so the script waits for path to complete.
result = client.moveOnPathAsync([airsim.Vector3r(pt[0], pt[1], pt[2]) for pt in path], 
                        12, 120, 
                        airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()

print("path complete, correct any overshoot at the end of the path")
print_position(client)

client.moveToPositionAsync(0,0,-10,1).join()
print("landing")
client.landAsync().join()
print("disarming drone")
client.armDisarm(False)
client.enableApiControl(False)
print("done!")

print_position(client)