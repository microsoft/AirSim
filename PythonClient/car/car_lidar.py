# Python client example to get Lidar data from a car
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
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

    def execute(self):

        for i in range(3):
           
            state = self.client.getCarState()
            s = pprint.pformat(state)
            #print("state: %s" % s)
            
            # go forward
            self.car_controls.throttle = 0.5
            self.car_controls.steering = 0
            self.client.setCarControls(self.car_controls)
            print("Go Forward")
            time.sleep(3)   # let car drive a bit

            # Go forward + steer right
            self.car_controls.throttle = 0.5
            self.car_controls.steering = 1
            self.client.setCarControls(self.car_controls)
            print("Go Forward, steer right")
            time.sleep(3)   # let car drive a bit
            
            airsim.wait_key('Press any key to get Lidar readings')
        
            for i in range(1,3):
                lidarData = self.client.getLidarData();
                if (len(lidarData.point_cloud) < 3):
                    print("\tNo points received from Lidar data")
                else:
                    points = self.parse_lidarData(lidarData)
                    print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                    print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                    print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
                time.sleep(5)

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def write_lidarData_to_disk(self, points):
        # TODO
        print("not yet implemented")

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes car move and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()
