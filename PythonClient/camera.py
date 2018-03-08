# use open cv to show new images from AirSim 

from AirSimClient import *
# requires Python 3.5.3 :: Anaconda 4.4.0
# pip install opencv-python
import cv2
import time
import sys
import numpy as np
import random as rnd

direction = []
def draw_flow(img, flow, step=16):
	h, w = img.shape[:2]
	y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
	fx, fy = flow[y,x].T
	lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
	lines = np.int32(lines + 0.5)
	vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
	cv2.polylines(vis, lines, 0, (0, 255, 0))
	for (x1, y1), (x2, y2) in lines:
		direction.append([x1,y1, math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))])
		cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
	return vis,lines

def calc(old_frame,new_frame):
	old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
	gray = cv2.cvtColor(new_frame, cv2.COLOR_BGR2GRAY)
	flow = cv2.calcOpticalFlowFarneback(old_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
	finalImg = draw_flow(gray,flow)
	finalImg,lines = draw_flow(gray,flow)
	return finalImg,lines

def printUsage():
   print("Usage: python camera.py [depth|segmentation|scene]")

cameraType = "scene"

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

client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
#client.armDisarm(True)
#client.takeoff()

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

img1 = None
img2 = None

time1 = time.time()
pos = [-10, 10, -10]
client.moveToPosition(pos[0], pos[1], pos[2], 5, 0)
while True:
    # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
    rawImage = client.simGetImage(3, cameraTypeMap[cameraType])
    if (rawImage == None):
        print("Camera is not returning image, please check airsim for error messages")
        sys.exit(0)
    else:
        png = cv2.imdecode(AirSimClientBase.stringToUint8Array(rawImage), cv2.IMREAD_UNCHANGED)
        
        #cv2.putText(png,'FPS ' + str(fps),textOrg, fontFace, fontScale,(255,255,255),thickness)
        #cv2.imshow("Depth", png)
        img2 = png
        if type(img1) == type(None):
            img1 = img2
        # calculate
        ans = calc(img1, img2)
        #cv2.imshow("Flow", ans[0])
        cv2.imshow("Flow", png)
        img1 = img2
    frameCount  = frameCount  + 1
    endTime=time.clock()
    diff = endTime - startTime
    if (diff > 1):
        fps = frameCount
        frameCount = 0
        startTime = endTime
    
    key = cv2.waitKey(1) & 0xFF
    if (key == 27 or key == ord('q') or key == ord('x')):
        break
    if time.time() - time1 > 10.0:
        # get new random place 
        pos = [rnd.randint(-10, 10), rnd.randint(-10, 10), rnd.randint(-10, -3)]
        time1 = time.time()
        client.moveToPosition(pos[0], pos[1], pos[2], 5, 0)
    currPos = client.simGetPose().position
    
    print( ((pos[0]-currPos.x_val)**2 + (pos[1]-currPos.y_val)**2 + (pos[2]-currPos.z_val)**2)**(1/2) )
    #client.rotateByYawRate(15, 0.1)

