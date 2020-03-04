import setup_path
import airsim

import time

# change clock speed in settings.json
# "ClockSpeed": 0.5

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.moveByVelocityZAsync(0, 0, -20, 3).join()


while True:
    client.moveByVelocityZAsync(5, 5, -2, 1).join()
    time.sleep(10)

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
