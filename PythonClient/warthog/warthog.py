#import setath
import airsim
import pandas as pd
import cv2
from pyquaternion import Quaternion
import numpy as np
import os
import time
import tempfile
import matplotlib.pyplot as plt
import pygame
import math
pygame.init()

def zero_to_2pi( theta):
    if theta < 0:
        theta = 2 * math.pi + theta
    elif theta > 2 * math.pi:
        theta = theta - 2 * math.pi
    return theta
#joystick = pygame.joystick.Joystick(0)
# connect to the AirSim simulator
client = airsim.WarthogClient(ip = "172.31.208.1")
client.confirmConnection()
client.enableApiControl(True)
print("API Control enabled: %s" % client.isApiControlEnabled())
war_controls = airsim.WarthogControls()
lin = []
ang = []
dlt = []
#ar_controls.linear_vel = 4.0
#ar_controls.angular_vel = 1.0
client.setWarthogControls(war_controls)
'''war_state = client.getWarthogState()
a = war_state.kinematics_estimated.orientation.to_numpy_array()
q = Quaternion(a[3], a[0], a[1], a[2])
init_angle = np.sign(q.axis[2])*q.degrees'''
not_done = True;
i = 0
while not_done:
    st_time = time.time()
    '''for event in pygame.event.get():
        pass
    a0 = joystick.get_axis(1)
    a1 = joystick.get_axis(2)
    button = joystick.get_button(0)
    if button == 1:
        not_done=False
        break'''
    a0 = 0.5
    a1= 0.5
    war_controls.linear_vel = -a0*5.0
    war_controls.angular_vel = -a1*2.0
    client.setWarthogControls(war_controls)
    war_state = client.getWarthogState()
    lin.append(war_state.linear_vel)
    ang.append(war_state.angular_vel)
    gps_data = client.getGpsData()
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    response = responses[0]
    if not i%10:
        a = war_state.kinematics_estimated.orientation.to_numpy_array()
        q = Quaternion(a[3], a[0], a[1], a[2])
        left_handed_system_pose = war_state.kinematics_estimated.position
        right_handed_system_pose = [left_handed_system_pose.x_val, -left_handed_system_pose.y_val]
        #print(f'full state {war_state.kinematics_estimated.orientation.to_numpy_array()}')
        #print(f'degrees = {np.sign(q.axis[2])*q.degrees}')
        yaw = zero_to_2pi(-np.sign(q.axis[2])*q.radians)*180/math.pi
        print(f'degrees = {yaw}')
        print(f'pose = {right_handed_system_pose}')
        #print(f'angle = {np.sign(q.axis[2])*q.angle}')
        #print(f'axis = {q.axis}')
    i = i+1

# get numpy array
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 

# reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)
    cv2.imshow("war", img_rgb)
    cv2.waitKey(1)
    dlt.append(time.time() - st_time)
    
   # print(gps_data)
   # time.sleep(0.01)

# get numpy array

# reshape array to 4 channel image array H X W X 4

# original image is fliped vertically
#img_rgb = np.flipud(img_rgb)

# write to png 
cv2.destroyAllWindows() 
#a = input("input key to stop")
#print(dlt)
#plt.figure()
#plt.plot(ang, 'r')
#plt.plot(lin, 'g')
#plt.figure()
#plt.show()'''
war_controls.linear_vel = 0
war_controls.angular_vel = 0
client.setWarthogControls(war_controls)
#a = input("input key to stop")
client.reset()
client.enableApiControl(False)

