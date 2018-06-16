from AirSimClient import *


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.moveByVelocityZAsync(0, 0, -2, 3)


while True:
    client.moveByVelocityZAsync(5, 5, -2, 1).join()
    time.sleep(10)

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
