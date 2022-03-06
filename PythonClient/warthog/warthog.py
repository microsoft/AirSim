import setup_path
import airsim
#import cv2
import numpy as np
import os
import time
import tempfile

# connect to the AirSim simulator
client = airsim.WarthogClient()
client.confirmConnection()
client.enableApiControl(True)
print("API Control enabled: %s" % client.isApiControlEnabled())
war_controls = airsim.WarthogControls()
war_controls.linear_vel = 4.
war_controls.angular_vel = 1.
client.setWarthogControls(war_controls)
a = input("input key to stop")
war_controls.linear_vel = 0
war_controls.angular_vel = 0
client.setWarthogControls(war_controls)
a = input("input key to stop")
client.reset()
client.enableApiControl(False)

