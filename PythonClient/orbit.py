from AirSimClient import *
import sys
import math
import time
import argparse

class Position:
    def __init__(self, pos):
        self.x = pos.x_val
        self.y = pos.y_val
        self.z = pos.z_val

# Make the drone fly in a circle.
class OrbitNavigator:
    def __init__(self, args):
        self.radius = args.radius
        self.altitude = args.altitude
        self.speed = args.speed
        self.iterations = args.iterations
        if self.iterations <= 0:
            self.iterations = 1

        p = args.center.split(',')
        if len(p) != 2:
            raise Exception("Expecting 'x,y' for the center direction vector")
        
        cx = float(p[0])
        cy = float(p[1])
        length = math.sqrt(cx*cx)+(cy*cy)
        cx /= length
        cy /= length
        cx *= self.radius
        cy *= self.radius

        self.client = MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        self.home = self.getPosition()
        # check that our home position is stable
        start = time.time()
        count = 0
        while count < 100:
            pos = self.home
            if abs(pos.z - self.home.z) > 1:                                
                count = 0
                home = pos
                if time.time() - start > 10:
                    print("Drone position is drifting, we are waiting for it to settle down...")
                    start = time
            else:
                count += 1

        self.center = self.getPosition()
        self.center.x += cx
        self.center.y += cy


    def getPosition(self):
        pos = self.client.getPosition()
        return Position(pos)

    def start(self):
        print("arming the drone...")
        self.client.armDisarm(True)
        
        # AirSim uses NED coordinates so negative axis is up.
        start = self.getPosition()
        z = -self.altitude + self.home.z
        landed = self.client.getLandedState()
        if landed == LandedState.Landed:            
            print("taking off...")
            self.client.takeoff()
        else:
            print("already flying so we will orbit at current altitude {}".format(start.z))
            z = start.z # use current altitude then
        
        print("climbing to position: {},{},{}".format(start.x, start.y, z))
        self.client.moveToPosition(start.x, start.y, z, self.speed)

        print("ramping up to speed...")
        count = 0
        self.start_angle = None
        
        # ramp up time
        ramptime = self.radius / 10
        start_time = time.time()        

        while count < self.iterations:

            # ramp up to full speed in smooth increments so we don't start too aggresively.
            now = time.time()
            speed = self.speed
            diff = now - start_time
            if diff < ramptime:
                speed = self.speed * diff / ramptime
            elif ramptime > 0:
                print("reached full speed...")
                ramptime = 0
                
            lookahead_angle = speed / self.radius            

            # compute current angle
            pos = self.getPosition()
            dx = pos.x - self.center.x
            dy = pos.y - self.center.y
            actual_radius = math.sqrt((dx*dx) + (dy*dy))
            angle_to_center = math.atan2(dy, dx)

            camera_heading = (angle_to_center - math.pi) * 180 / math.pi 

            # compute lookahead
            lookahead_x = self.center.x + self.radius * math.cos(angle_to_center + lookahead_angle)
            lookahead_y = self.center.y + self.radius * math.sin(angle_to_center + lookahead_angle)

            vx = lookahead_x - pos.x
            vy = lookahead_y - pos.y

            if self.track_orbits(angle_to_center * 180 / math.pi):
                count += 1
                print("completed {} orbits".format(count))
            
            self.client.moveByVelocityZ(vx, vy, z, 1, DrivetrainType.MaxDegreeOfFreedom, YawMode(False, camera_heading))
            
        if z < self.home.z:
            print("descending")
            self.client.moveToPosition(start.x, start.y, self.home.z - 5, 2)

        print("landing...")
        self.client.land()

        print("disarming.")
        self.client.armDisarm(False)

    def track_orbits(self, angle):
        # tracking # of completed orbits is surprisingly tricky to get right in order to handle random wobbles
        # about the starting point.  So we watch for complete 1/2 orbits to avoid that problem.
        if angle < 0:
            angle += 360

        if self.start_angle is None:
            self.start_angle = angle
            self.previous_angle = angle
            self.shifted = False
            self.previous_sign = None
            self.previous_diff = None            
            self.quarter = False
            return False

        # now we just have to watch for a smooth crossing from negative diff to positive diff
        if self.previous_angle is None:
            self.previous_angle = angle
            return False

        diff = self.previous_angle - angle
        crossing = False
        self.previous_angle = angle

        # ignore any large discontinuities 
        if abs(diff) > 45:
            return False
    
        diff = abs(angle - self.start_angle)
        if diff > 45:
            self.quarter = True

        if self.quarter and self.previous_diff is not None and diff != self.previous_diff:
            # watch direction this diff is moving if it switches from shrinking to growing
            # then we passed the starting point.
            direction = self.sign(self.previous_diff - diff)
            if self.previous_sign is None:
                self.previous_sign = direction
            elif self.previous_sign > 0 and direction < 0:
                if diff < 45:
                    crossing = True
                    self.quarter = False
            self.previous_sign = direction
        self.previous_diff = diff

        return crossing

    def sign(self, s):
        if s < 0: 
            return -1
        return 1

if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("Orbit.py makes drone fly in a circle with camera pointed at the given center vector")
    arg_parser.add_argument("--radius", type=float, help="radius of the orbit", default=30)
    arg_parser.add_argument("--altitude", type=float, help="altitude of orbit (in positive meters)", default=30)
    arg_parser.add_argument("--speed", type=float, help="speed of orbit (in meters/second)", default=5)
    arg_parser.add_argument("--center", help="x,y direction vector pointing to center of orbit from current starting position (default 1,0)", default="1,0")
    arg_parser.add_argument("--iterations", type=float, help="number of 360 degree orbits (default 1)", default=3)
    args = arg_parser.parse_args(args)
    nav = OrbitNavigator(args)
    nav.start()
