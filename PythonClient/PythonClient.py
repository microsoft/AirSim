# to install this run 'pip install msgpack-rpc-python'
import msgpackrpc
import numpy as np
import math;

class AirSimImageType:    
    Off = 0
    Scene = 1 
    Depth = 2
    Segmentation = 4

class DrivetrainType:
    MaxDegreeOfFreedom = 0
    ForwardOnly = 1
    
class LandedState:
    Landed = 0
    Flying = 1

class YawMode:
    is_rate = True
    yaw_or_rate = 0.0
    def __init__(self, is_rate, yaw_or_rate):
        self.is_rate = is_rate
        self.yaw_or_rate = yaw_or_rate

class AirSimClient:
        def __init__(self, ip):
            if (ip == ""):
                ip = "127.0.0.1"
            self.client = msgpackrpc.Client(msgpackrpc.Address(ip, 41451), timeout = 3600)

        # basic flight control
        def arm(self):
            return self.client.call('armDisarm', True)
        
        def disarm(self):
            return self.client.call('armDisarm', False)

        def takeoff(self, max_wait_seconds):
            return self.client.call('takeoff', max_wait_seconds)
        
        def land(self, max_wait_seconds):
            return self.client.call('land', max_wait_seconds)
        
        def goHome(self):
            return self.client.call('goHome')

        def hover(self):
            return self.client.call('hover')

        
        # query position of drone
        def getPosition(self):
            return self.client.call('getPosition')
        def getVelocity(self):
            return self.client.call('getVelocity')
        def getOrientation(self):
            return self.client.call('getOrientation')
        def getLandedState(self):
            return self.client.call('getLandedState')
        def getGpsLocation(self):
            return self.client.call('getGpsLocation')
        def getHomePoint(self):
            return self.client.call('getHomePoint')
        def getRollPitchYaw(self):
            return self.toEulerianAngle(self.getOrientation())

        # special offboard control
        def isOffboardMode(self):
            return self.client.call('isOffboardMode')
        def moveByVelocity(self, vx, vy, vz, duration, drivetrain, yaw_mode):
            return self.client.call('moveByVelocity', vx, vy, vz, duration, drivetrain, (yaw_mode.is_rate, yaw_mode.yaw_or_rate))
        def moveByVelocityZ(self, vx, vy, z, duration, drivetrain, yaw_mode):
            return self.client.call('moveByVelocityZ', vx, vy, z, duration, drivetrain, (yaw_mode.is_rate, yaw_mode.yaw_or_rate))
        def moveOnPath(self, path, velocity, max_wait_seconds, drivetrain, yaw_mode, lookahead, adaptive_lookahead):
            return self.client.call('moveOnPath', path, velocity, max_wait_seconds, drivetrain, (yaw_mode.is_rate, yaw_mode.yaw_or_rate), lookahead, adaptive_lookahead)
        def moveToZ(self, z, velocity, max_wait_seconds, yaw_mode, lookahead, adaptive_lookahead):
            return self.client.call('moveToZ', z, velocity, max_wait_seconds, (yaw_mode.is_rate, yaw_mode.yaw_or_rate), lookahead, adaptive_lookahead)
        def moveToPosition(self, x, y, z, velocity, max_wait_seconds, drivetrain, yaw_mode, lookahead, adaptive_lookahead):
            return self.client.call('moveToPosition', x, y, z, velocity, max_wait_seconds, drivetrain, (yaw_mode.is_rate, yaw_mode.yaw_or_rate), lookahead, adaptive_lookahead)
        def rotateToYaw(self, yaw, max_wait_seconds, margin):
            return self.client.call('rotateToYaw', yaw, max_wait_seconds, margin)
        def rotateByYawRate(self, yaw_rate, duration):
            return self.client.call('rotateByYawRate', yaw_rate, duration)

        # camera control
        # getImageForCamera returns compressed png in array of bytes
        # image_type uses one of the AirSimImageType members
        def getImageForCamera(self, camera_id, image_type):
            # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
            result = self.client.call('getImageForCamera', camera_id, image_type)
            if (result == "" or result == "\0"):
                return None
            return np.fromstring(result, np.int8)

        # helper method for converting getOrientation to roll/pitch/yaw
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        def toEulerianAngle(self, q):
            x = q[0]
            y = q[1]
            z = q[2]
            w = q[3]
            ysqr = y * y;

            # roll (x-axis rotation)
            t0 = -2.0* (ysqr + z * z) + 1.0;
            t1 = +2.0* (x * y + w * z);
            roll = math.atan2(t1, t0)

            # pitch (y-axis rotation)
            t2 = -2.0* (x * z - w * y);
            if (t2 > 1.0):
                t2 = 1
            if (t2 < -1.0):
                t2 = -1.0
            pitch = math.sin(t2)

            # yaw (z-axis rotation)
            t3 = +2.0* (y * z + w * x);
            t4 = -2.0* (x * x + ysqr) + 1.0;
            yaw = math.atan2(t3, t4)

            return (pitch, roll, yaw)

