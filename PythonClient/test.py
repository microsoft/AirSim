from PythonClient import *
import sys
import time

client = AirSimClient('127.0.0.1')

print("Waiting for home GPS location to be set...")
home = client.getHomePoint()
while (home[0] == 0 and home[1] == 0 and home[2] == 0):
    time.sleep(1)
    home = client.getHomePoint()

print("Home lat=%g, lon=%g, alt=%g" % home)

if (not client.arm()):
    print("failed to arm the drone")
    sys.exit(1);

if (not client.takeoff()):
    print("failed to takeoff")
    sys.exit(1);

print("Should now be flying...")
time.sleep(5)

if (not client.land()):
    print("failed to land")
    sys.exit(1);



