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

    def __init__(self, save_to_disk, change_time_of_day):

        self.save_to_disk = save_to_disk
        self.change_time_of_day = change_time_of_day

        # connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        
        if (self.change_time_of_day):
            time_of_day = "2018-11-27 8:00:00"
            airsim.wait_key('Press any key to change time of day to [{}]'.format(time_of_day))
            self.client.simSetTimeOfDay(True, time_of_day)
            time.sleep(3)

    def execute(self):

        for i in range(3):

            if (self.change_time_of_day):
                time_of_day = "2018-11-27 {}:00:00".format(12+ (i * 3))
                airsim.wait_key('Press any key to change time of day to [{}]'.format(time_of_day))
                self.client.simSetTimeOfDay(True, time_of_day)

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
        
        if (self.change_time_of_day):
            airsim.wait_key('Press any key to reset time of day')
            self.client.simSetTimeOfDay(False)

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
    arg_parser.add_argument('-change-time-of-day', type=bool, help="change time of day", default=False)

    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest(args.save_to_disk, args.change_time_of_day)
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()
