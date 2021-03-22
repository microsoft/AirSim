# Python client example to get Lidar data from a drone
# This script is for Lidar sensors using 'SensorLocalFrame' as DataFrame under settings.json

import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy as np

# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        print('Connected!\n')

    def execute(self,vehicleName,lidarNames):
        #self.client.reset()
        print('Scanning Has Started\n')
        print('Use Keyboard Interrupt \'CTRL + C\' to Stop Scanning\n')
        flag1=0
        try:
            while True:
                for lidars in lidarNames:
                    if flag1 < len(lidarNames):
                        f = open(vehicleName+'_'+lidars+'_pointcloud.asc','w')
                    else:
                        f = open(vehicleName+'_'+lidars+'_pointcloud.asc','a')
                    lidarData = self.client.getLidarData(lidar_name=lidars,vehicle_name=vehicleName);
                    [q0,q1,q2,q3] = [lidarData.pose.orientation.w_val,lidarData.pose.orientation.x_val,lidarData.pose.orientation.y_val,lidarData.pose.orientation.z_val]
                    rotationMatrix = np.array(([1-2*(q2*q2+q3*q3),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)],
                                                  [2*(q1*q2+q3*q0),1-2*(q1*q1+q3*q3),2*(q2*q3-q1*q0)],
                                                  [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1*q1+q2*q2)]))

                    flag = 0
                    XYZ = np.zeros((3,1))
                    for item in lidarData.point_cloud:
                        if (flag == 0):
                            XYZ[0,0] = item
                        elif (flag == 1):
                            XYZ[1,0] = item
                        else:
                            XYZ[2,0] = item
                        flag = flag + 1
                        if (flag == 3):
                            [correctedX,correctedY,correctedZ] = np.matmul(rotationMatrix,XYZ)
                            finalX = correctedX + lidarData.pose.position.x_val
                            finalY = correctedY + lidarData.pose.position.y_val
                            finalZ = correctedZ + lidarData.pose.position.z_val
                            f.write("%f %f %f %d %d %d \n" % (finalX,finalY,finalZ,255,255,0))
                            flag=0
                        else:
                            pass
                    f.close()
                    flag1=flag1+1
        except KeyboardInterrupt:
            pass
            #time.sleep(5)
            airsim.wait_key('Press any key to reset to original state')
            self.client.reset()
            print("Done!\n")

# main
if __name__ == "__main__":
    lidarTest = LidarTest()
    lidarTest.execute('Drone1',['LidarSensor1','LidarSensor2'])
