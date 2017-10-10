# use open cv to create point cloud from depth image.
from AirSimClient import *
import cv2
import time
import sys
import math
import numpy as np

outputFile = "cloud.asc"
color = (0,255,0)
rgb = "%d %d %d" % color
projectionMatrix = np.array([[-0.501202762, 0.000000000, 0.000000000, 0.000000000],
                              [0.000000000, -0.501202762, 0.000000000, 0.000000000],
                              [0.000000000, 0.000000000, 10.00000000, 100.00000000],
                              [0.000000000, 0.000000000, -10.0000000, 0.000000000]])


def printUsage():
   print("Usage: python point_cloud.py [cloud.txt]")
   
def savePointCloud(image, fileName):
   f = open(fileName, "w")
   for x in xrange(image.shape[0]):
     for y in xrange(image.shape[1]):
        pt = image[x,y]
        if (math.isinf(pt[0]) or math.isnan(pt[0])):
          # skip it
          None
        else: 
          f.write("%f %f %f %s\n" % (pt[0], pt[1], pt[2]-1, rgb))
   f.close()

for arg in sys.argv[1:]:
  cloud.txt = arg

client = AirSimClient('127.0.0.1')

while True:
    rawImage = client.getImageForCamera(0, AirSimImageType.Depth)
    if (rawImage is None):
        print("Camera is not returning image, please check airsim for error messages")
        sys.exit(0)
    else:
        png = cv2.imdecode(rawImage, cv2.IMREAD_UNCHANGED)
        gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)
        Image3D = cv2.reprojectImageTo3D(gray, projectionMatrix)
        savePointCloud(Image3D, outputFile)
        print("saved " + outputFile)
        sys.exit(0)

    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        break;
