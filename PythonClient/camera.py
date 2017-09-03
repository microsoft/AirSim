# use open cv to show new images from AirSim 

from PythonClient import *
# requires Python 3.5.3 :: Anaconda 4.4.0
# pip install opencv-python
import cv2
import time
import sys

def printUsage():
   print("Usage: python camera.py [depth|segmentation|scene]")

cameraType = "depth"

for arg in sys.argv[1:]:
  cameraType = arg.lower()

cameraTypeMap = { 
 "depth": AirSimImageType.DepthVis,
 "segmentation": AirSimImageType.Segmentation,
 "seg": AirSimImageType.Segmentation,
 "scene": AirSimImageType.Scene,
 "disparity": AirSimImageType.DisparityNormalized,
 "normals": AirSimImageType.SurfaceNormals
}

if (not cameraType in cameraTypeMap):
  printUsage()
  sys.exit(0)

print (cameraTypeMap[cameraType])

client = AirSimClient('127.0.0.1')
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoff()

help = False

fontFace = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.5
thickness = 2
textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)
print (textSize)
textOrg = (10, 10 + textSize[1])
frameCount = 0
startTime=time.clock()
fps = 0

while True:
    # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
    rawImage = client.simGetImage(0, cameraTypeMap[cameraType])
    if (rawImage == None):
        print("Camera is not returning image, please check airsim for error messages")
        sys.exit(0)
    else:
        png = cv2.imdecode(AirSimClient.stringToUint8Array(rawImage), cv2.IMREAD_UNCHANGED)
        cv2.putText(png,'FPS ' + str(fps),textOrg, fontFace, fontScale,(255,0,255),thickness)
        cv2.imshow("Depth", png)

    frameCount  = frameCount  + 1
    endTime=time.clock()
    diff = endTime - startTime
    if (diff > 1):
        fps = frameCount
        frameCount = 0
        startTime = endTime
    
    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        break;
