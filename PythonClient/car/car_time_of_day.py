# Python client example to change time-of-day using APIs
# 

import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy

# Changes time of the day and makes the car move around
class TimeOfDayTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

    def execute(self):

        for i in range(8):
            
            # flip between specific time and default time
            enabled = False
            if (i % 2 == 0):
                enabled = True

            self.setTimeOfDay(enabled, "2018-11-27 {}:00:00".format(8+ (i * 2)))

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

    def setTimeOfDay(self, enabled, time_of_day):
            if (enabled):
                airsim.wait_key('Press any key to change time of day to [{}]'.format(time_of_day))
                self.client.simSetTimeOfDay(enabled, time_of_day)
            else:
                airsim.wait_key('Press any key to change time of day to default time')
                self.client.simSetTimeOfDay(enabled)

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("TimeOfDay.py changes time of the day")
  
    args = arg_parser.parse_args(args)    
    timeOfDayTest = TimeOfDayTest()
    try:
        timeOfDayTest.execute()
    finally:
        timeOfDayTest.stop()
