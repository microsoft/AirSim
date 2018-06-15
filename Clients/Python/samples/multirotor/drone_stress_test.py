from AirSimClient import *

# connect to the AirSim simulator 
client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)


for idx in range(3000):
    client.moveToPosition(-10, 10, -10, 5)
    client.reset()
    client.enableApiControl(True)
    print("%d" % idx)

# that's enough fun for now. let's quite cleanly
client.enableApiControl(False)
