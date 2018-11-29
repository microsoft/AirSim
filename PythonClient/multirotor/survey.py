import setup_path 
import airsim

import sys
import time
import argparse

class SurveyNavigator:
    def __init__(self, args):
        self.boxsize = args.size
        self.stripewidth = args.stripewidth
        self.altitude = args.altitude
        self.velocity = args.speed
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def move_to_position(self, x, y, z, threshold=2):
        self.client.moveToPositionAsync(0, 0, z, self.velocity).join()
        # bufbuf: seems moveToPositionAsync isn't working right...
        while True:
            k = self.client.simGetGroundTruthKinematics()
            pos = self.client.getPosition()
            dx = abs(pos.x_val - x)
            dy = abs(pos.y_val - y)
            dz = abs(pos.z_val - z)
            if dx+dy+dz < threshold:
                return
            time.sleep(0.1)  # 100ms should be fine        

    def start(self):
        print("arming the drone...")
        self.client.armDisarm(True)

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("taking off...")
            self.client.takeoffAsync().join()
        
        # AirSim uses NED coordinates so negative axis is up.
        x = -self.boxsize
        z = -self.altitude

        print("climbing to altitude: " + str(self.altitude))
        self.move_to_position(0, 0, z)

        print("flying to first corner of survey box")
        self.move_to_position(x, -self.boxsize, z)
        
        # let it settle there a bit.
        self.client.hoverAsync().join()
        time.sleep(2)

        # now compute the survey path required to fill the box 
        path = []
        distance = 0
        while x < self.boxsize:
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            x += self.stripewidth            
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, -self.boxsize, z)) 
            x += self.stripewidth  
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, -self.boxsize, z))
            distance += self.boxsize 
        
        print("starting survey, estimated distance is " + str(distance))
        trip_time = distance / self.velocity
        print("estimated survey time is " + str(trip_time))
        try:
            result = self.client.moveOnPathAsync(path, self.velocity, trip_time, airsim.DrivetrainType.ForwardOnly, 
                airsim.YawMode(True,0), self.velocity + (self.velocity/2), 1).join()
        except:
            errorType, value, traceback = sys.exc_info()
            print("moveOnPath threw exception: " + str(value))
            pass

        print("flying back home")
        self.move_to_position(0, 0, z)
        
        if z < -5:
            print("descending")
            self.velocity = 2
            self.move_to_position(0, 0, -5)

        print("landing...")
        self.client.landAsync().join()

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
