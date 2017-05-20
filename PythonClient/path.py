from PythonClient import *
import sys
import time

client = AirSimClient('127.0.0.1')

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo


# this method is async and we are not waiting for the result (no call to get()).
# instead we attach an async callback method which is invoked when the path completes.
print("client.moveOnPath to fly fast path along the streets")
result = client.moveOnPath([(0,-253,z),(125,-253,z),(125,0,z),(0,0,z)], 12, 0, DrivetrainType.ForwardOnly, YawMode(False,0), 15, 1)

time.sleep(2)

print("Notice we passed zero for max_wait_seconds, which means the script will not wait for path to complete");
print("This means while drone is flying this path we can change our mind and call moveOnPath as ")
print("often as we want and it will cancel the previous path and continue with the new one.")  
result = client.moveOnPath([(0,-253,z),(125,-253,z),(125,0,z),(0,0,z)], 12, 0, DrivetrainType.ForwardOnly, YawMode(False,0), 15, 1)
