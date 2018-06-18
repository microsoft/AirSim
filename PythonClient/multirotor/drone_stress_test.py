import setup_path 
import airsim

# connect to the AirSim simulator 
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)


for idx in range(3000):
    client.moveToPositionAsync(0, 0, -10, 5).join()
    client.reset()
    client.enableApiControl(True)
    print("%d" % idx)

# that's enough fun for now. let's quite cleanly
client.enableApiControl(False)
