import setup_path
import airsim

import time

# Run this script with clock speed in settings.json
# "ClockSpeed": 1 then change it to 0.5

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

vx = vy = 0
z = -20
duration = 3

# with ClockSpeed = 0.5 you will see that this takes 6s (system time) and not 3s
client.moveByVelocityZAsync(vx, vy, z, duration).join()


while True:
    vx = vy = 5
    z = -20
    duration = 5
    # with ClockSpeed = 0.5 you will see that this takes 10s (system time)
    #and not 5s in each iteration
    client.moveByVelocityZAsync(vx, vy, z, duration).join()
    time.sleep(10)

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
