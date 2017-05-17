# to install this run 'pip install mprpc'
from mprpc import RPCClient
import numpy as np

class AirSimImageType:    
    Scene = 1 
    Depth = 2
    Segmentation = 4

class DrivetrainType:
    MaxDegreeOfFreedom = 0
    ForwardOnly = 1

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
            self.client = RPCClient(ip, 41451)

        # basic flight control
        def arm(self):
            return self.client.call('armDisarm', True)
        
        def disarm(self):
            return self.client.call('armDisarm', False)

        def takeoff(self):
            return self.client.call('takeoff', 10)
        
        def land(self):
            return self.client.call('land')
        
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
        def getGpsLocation(self):
            return self.client.call('getGpsLocation')
        def getHomePoint(self):
            return self.client.call('getHomePoint')

        # special offboard control
        def isOffboardMode(self):
            return self.client.call('isOffboardMode')
        def moveByVelocity(self, vx, vy, vz, duration, drivetrain, yaw_mode):
            return self.client.call('moveByVelocity', vx, vy, vz, duration, drivetrain, (yaw_mode.is_rate, yaw_mode.yaw_or_rate))
        def moveByVelocityZ(self, vx, vy, z, duration, drivetrain, yaw_mode):
            return self.client.call('moveByVelocityZ', vx, vy, z, duration, drivetrain, (yaw_mode.is_rate, yaw_mode.yaw_or_rate))
        def moveOnPath(self, path, velocity, drivetrain, yaw_mode, lookahead, adaptive_lookahead):
            return self.client.call('moveOnPath', path, velocity, drivetrain, (yaw_mode.is_rate, yaw_mode.yaw_or_rate), lookahead, adaptive_lookahead)
        def moveToZ(self, z, velocity, yaw_mode, lookahead, adaptive_lookahead):
            return self.client.call('moveToZ', z, velocity, (yaw_mode.is_rate, yaw_mode.yaw_or_rate), lookahead, adaptive_lookahead)
        def rotateToYaw(self, yaw, margin):
            return self.client.call('rotateToYaw', yaw, margin)
        def rotateByYawRate(self, yaw_rate, duration):
            return self.client.call('rotateByYawRate', yaw_rate, duration)

        # camera control
        # image_type uses one of the AirSimImageType members
        def setImageTypeForCamera(self, camera_id, image_type):
            return self.client.call('setImageTypeForCamera', camera_id, image_type)
        def getImageTypeForCamera(self, camera_id):
            return self.client.call('getImageTypeForCamera', camera_id)
        def getImageForCamera(self, camera_id, image_type):
            return np.fromstring(self.client.call('getImageForCamera', camera_id, image_type), np.int8)


