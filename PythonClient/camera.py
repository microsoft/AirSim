# use open cv to show new images from AirSim 

from PythonClient import *
import cv2
import time
import sys

client = AirSimClient('127.0.0.1')

# get depth image
result = client.setImageTypeForCamera(0, AirSimImageType.Depth)

time.sleep(1) # give it time to render
help = False

while True:
    # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
    result = client.getImageForCamera(0, AirSimImageType.Depth)
    if (result == "\0"):
        if (not help):
            help = True
            print("Please press '1' in the AirSim view to enable the Depth camera view")
    else:
        rawImage = np.fromstring(result, np.int8)
        png = cv2.imdecode(rawImage, cv2.IMREAD_UNCHANGED)
        cv2.imshow("Traffic", png)

    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        break;
