import setup_path
import airsim

import sys
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

print("Flying a small square box using moveByVelocityZ")

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7

# Fly given velocity vector for 5 seconds
duration = 5
speed = 1
delay = duration * speed

# using airsim.DrivetrainType.MaxDegreeOfFreedom means we can control the drone yaw independently
# from the direction the drone is flying.  I've set values here that make the drone always point inwards
# towards the inside of the box (which would be handy if you are building a 3d scan of an object in the real world).
vx = speed
vy = 0
print("moving by velocity vx=" + str(vx) + ", vy=" + str(vy) + ", yaw=90")
client.moveByVelocityZAsync(vx,vy,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 90)).join()
time.sleep(delay)
vx = 0
vy = speed
print("moving by velocity vx=" + str(vx) + ", vy=" + str(vy)+ ", yaw=180")
client.moveByVelocityZAsync(vx,vy,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 180)).join()
time.sleep(delay)
vx = -speed
vy = 0
print("moving by velocity vx=" + str(vx) + ", vy=" + str(vy)+ ", yaw=270")
client.moveByVelocityZAsync(vx, vy, z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 270)).join()
time.sleep(delay)
vx = 0
vy = -speed
print("moving by velocity vx=" + str(vx) + ", vy=" + str(vy) + ", yaw=0")
client.moveByVelocityZAsync(vx, vy,z,duration, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()
time.sleep(delay)
client.hoverAsync().join()
client.landAsync().join()
