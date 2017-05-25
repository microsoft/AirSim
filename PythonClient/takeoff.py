from PythonClient import *
import sys
import time
import msgpackrpc
import math

client = AirSimClient('127.0.0.1')

print("Waiting for home GPS location to be set...")
home = client.getHomePoint()
while ((home[0] == 0 and home[1] == 0 and home[2] == 0) or
       math.isnan(home[0]) or  math.isnan(home[1]) or  math.isnan(home[2])):
    time.sleep(1)
    home = client.getHomePoint()

print("Home lat=%g, lon=%g, alt=%g" % tuple(home))

if (not client.arm()):
    print("failed to arm the drone")
    sys.exit(1);

if (client.getLandedState() == LandedState.Landed):
    print("Taking off...")
    if (not client.takeoff(60)):
        print("failed to reach takeoff altitude after 60 seconds")
        sys.exit(1);
    print("Should now be flying...")
else:
    print("it appears the drone is already flying")

client.hover();
time.sleep(5)

