from AirSimClient import *
import sys
import time
import argparse

class SurveyNavigator:
    def __init__(self, args):
        self.boxsize = args.size
        self.stripewidth = args.stripewidth
        self.altitude = args.altitude
        self.velocity = args.speed
        self.client = MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def start(self):
        print("arming the drone...")
        self.client.armDisarm(True)

        landed = self.client.getLandedState()
        if landed != 1:
            print("taking off...")
            self.client.takeoff()
        
        # AirSim uses NED coordinates so negative axis is up.
        x = -self.boxsize
        z = -self.altitude

        print("climing to altitude: " + str(self.altitude))
        self.client.moveToPosition(0, 0, z, self.velocity)

        print("flying to first corner of survey box")
        self.client.moveToPosition(x, -self.boxsize, z, self.velocity)
        
        # let it settle there a bit.
        self.client.hover()
        time.sleep(2)

        # now compute the survey path required to fill the box 
        path = []
        distance = 0
        while x < self.boxsize:
            distance += self.boxsize 
            path.append(Vector3r(x, self.boxsize, z))
            x += self.stripewidth            
            distance += self.stripewidth 
            path.append(Vector3r(x, self.boxsize, z))
            distance += self.boxsize 
            path.append(Vector3r(x, -self.boxsize, z)) 
            x += self.stripewidth  
            distance += self.stripewidth 
            path.append(Vector3r(x, -self.boxsize, z))
            distance += self.boxsize 
        
        print("starting survey, estimated distance is " + str(distance))
        trip_time = distance / self.velocity
        print("estimated survey time is " + str(trip_time))
        try:
            self.client.enableApiControl(True)
            result = self.client.moveOnPath(path, self.velocity, trip_time, DrivetrainType.ForwardOnly, YawMode(False,0), self.velocity + (self.velocity/2), 1)
        except:
            errorType, value, traceback = sys.exc_info()
            print("moveOnPath threw exception: " + str(value))
            pass

        print("flying back home")
        self.client.moveToPosition(0, 0, z, self.velocity)
        
        if z < -5:
            print("descending")
            self.client.moveToPosition(0, 0, -5, 2)

        print("landing...")
        self.client.land()

        print("disarming.")
        self.client.armDisarm(False)

if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("Usage: survey boxsize stripewidth altitude")
    arg_parser.add_argument("--size", type=float, help="size of the box to survey", default=50)
    arg_parser.add_argument("--stripewidth", type=float, help="stripe width of survey (in meters)", default=10)
    arg_parser.add_argument("--altitude", type=float, help="altitude of survey (in positive meters)", default=30)
    arg_parser.add_argument("--speed", type=float, help="speed of survey (in meters/second)", default=5)
    args = arg_parser.parse_args(args)
    nav = SurveyNavigator(args)
    nav.start()
