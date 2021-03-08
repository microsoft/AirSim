# Python client example to get Lidar data from a drone
#

import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy

# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def execute(self):
        #self.client.reset()
        print('Scanning Has Started')
        f = open('data.asc', "w")
        try:
            while True:
                lidarData = self.client.getLidarData();
                [q0,q1,q2,q3] = [lidarData.pose.orientation.w_val,lidarData.pose.orientation.x_val,lidarData.pose.orientation.y_val,lidarData.pose.orientation.z_val]
                rotationMatrix = numpy.array(([1-2*(q2*q2+q3*q3),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)],
                                              [2*(q1*q2+q3*q0),1-2*(q1*q1+q3*q3),2*(q2*q3-q1*q0)],
                                              [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1*q1+q2*q2)]))

                flag = 0
                XYZ = numpy.zeros((3,1))
                for item in lidarData.point_cloud:
                    if (flag == 0):
                        XYZ[0,0] = item
                    elif (flag == 1):
                        XYZ[1,0] = item
                    else:
                        XYZ[2,0] = item
                    flag = flag + 1
                    if (flag == 3):
                        [correctedX,correctedY,correctedZ] = numpy.matmul(rotationMatrix,XYZ)
                        finalX = correctedX + lidarData.pose.position.x_val
                        finalY = correctedY + lidarData.pose.position.y_val
                        finalZ = correctedZ + lidarData.pose.position.z_val
                        f.write("%f %f %f %d %d %d \n" % (finalX,finalY,finalZ,255,255,0))
                        flag=0
                    else:
                        pass
        except KeyboardInterrupt:
            f.close()
            pass
            #time.sleep(5)

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()
