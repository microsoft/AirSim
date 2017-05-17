from PythonClient import *
import sys
import time

client = AirSimClient('127.0.0.1')


print("Flying a small square box using moveByVelocityZ")
client.moveByVelocityZ(1,0,-5,5, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, 0))
time.sleep(2)
client.moveByVelocityZ(0,1,-5,5, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, 90))
time.sleep(2)
client.moveByVelocityZ(-1,0,-5,5, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, 180))
time.sleep(2)
client.moveByVelocityZ(0,-1,-5,5, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, 270))
time.sleep(2)

