import setup_path
import airsim
#import cv2
import numpy as np
import os
import time
import tempfile
import matplotlib.pyplot as plt
import pygame
pygame.init()
joystick = pygame.joystick.Joystick(0)
# connect to the AirSim simulator
client = airsim.WarthogClient()
client.confirmConnection()
client.enableApiControl(True)
print("API Control enabled: %s" % client.isApiControlEnabled())
war_controls = airsim.WarthogControls()
lin = []
ang = []
#ar_controls.linear_vel = 4.0
#ar_controls.angular_vel = 1.0
client.setWarthogControls(war_controls)
for i in range(0,10000):
    for event in pygame.event.get():
        pass
    a0 = joystick.get_axis(1)
    a1 = joystick.get_axis(2)
    war_controls.linear_vel = -a0*4.0
    war_controls.angular_vel = -a1
    client.setWarthogControls(war_controls)
    war_state = client.getWarthogState()
    lin.append(war_state.linear_vel)
    ang.append(war_state.angular_vel)
    time.sleep(0.01)
a = input("input key to stop")
plt.figure()
plt.plot(ang, 'r')
plt.plot(lin, 'g')
#plt.figure()
plt.show()
war_controls.linear_vel = 0
war_controls.angular_vel = 0
client.setWarthogControls(war_controls)
a = input("input key to stop")
client.reset()
client.enableApiControl(False)

