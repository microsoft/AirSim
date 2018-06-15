from AirSimClient import *


# connect to the AirSim simulator
client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("fly")
client.moveToPosition(0, 0, -10, 5)
time.sleep(5)   # let car drive a bit

print("reset")
client.reset()
client.enableApiControl(True)
client.armDisarm(True)
time.sleep(5)   # let car drive a bit


print("fly")
client.moveToPosition(0, 0, -10, 5)
time.sleep(5)   # let car drive a bit