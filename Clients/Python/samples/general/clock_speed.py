from AirSimClient import *


# connect to the AirSim simulator
client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.moveByVelocityZ(0, 0, -2, 3)


while True:
    client.moveByVelocityZ(5, 5, -2, 1)
    time.sleep(10)

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
