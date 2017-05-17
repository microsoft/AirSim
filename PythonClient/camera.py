# use open cv to show new images from AirSim 

from PythonClient import *
import cv2
import time

client = AirSimClient('127.0.0.1')

# you must first press "1" in the AirSim view to turn on the depth capture

# get depth image
client.setImageTypeForCamera(0, AirSimImageType.Depth)
time.sleep(1) # give it time to render

while True:
    rawImage = client.getImageForCamera(0, AirSimImageType.Depth)       
    png = cv2.imdecode(rawImage, cv2.IMREAD_UNCHANGED)
    cv2.imshow("Traffic", png)

    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        break;
