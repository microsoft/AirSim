from PythonClient import *
import sys
import time

client = AirSimClient('127.0.0.1')

print("Flying a small square box using moveByVelocityZ")
print("Try pressing 't' in the AirSim view to see a pink trace of the flight")

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7

# Fly given velocity vector for 5 seconds
duration = 5
speed = 1
delay = duration * speed;

# using DrivetrainType.MaxDegreeOfFreedom means we can control the drone yaw independently
# from the direction the drone is flying.  I've set values here that make the drone always point inwards
# towards the inside of the box (which would be handy if you are building a 3d scan of an object in the real world).
client.moveByVelocityZ(speed,0,z,duration, delay, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, 90))
client.moveByVelocityZ(0,speed,z,duration, delay, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, 180))
client.moveByVelocityZ(-speed,0,z,duration, delay, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, 270))
client.moveByVelocityZ(0,-speed,z,duration, delay, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, 0))
client.hover()

