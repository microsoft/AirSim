# use open cv to create point cloud from depth image.

from PythonClient import *
import cv2
import time
import sys

outputFile = "cloud.txt"
color = (0,255,0)
rgb = "%d %d %d" % color

def printUsage():
   print("Usage: python point_cloud.py [cloud.txt]")
   
def savePointCloud(image, fileName):
   f = open(fileName, "w")
   for x in xrange(image.shape[0]):
     for y in xrange(image.shape[1]):
        pt = image[x,y]
        print pt
        f.write("%f %f %f %s\n" % (pt[0], pt[1], pt[2], rgb))
   f.close()

for arg in sys.argv[1:]:
  cloud.txt = arg

client = AirSimClient('127.0.0.1')

while True:
    rawImage = client.getImageForCamera(0, AirSimImageType.Depth)
    if (rawImage == None):
        print("Camera is not returning image, please check airsim for error messages")
        sys.exit(0)
    else:
        png = cv2.imdecode(rawImage, cv2.IMREAD_UNCHANGED)
        savePointCloud(png, outputFile)
        print("saved " + outputFile)
        sys.exit(0)

    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        break;
