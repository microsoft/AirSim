from __future__ import print_function

from .utils import *
from .types import *

import msgpackrpc #install as admin: pip install msgpack-rpc-python
import numpy as np #pip install numpy
import msgpack
import time
import math

class VehicleClient:
    def __init__(self, ip = "", port = 41451, timeout_value = 3600):
        if (ip == ""):
            ip = "127.0.0.1"
        self.client = msgpackrpc.Client(msgpackrpc.Address(ip, port), timeout = timeout_value, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')
        
    # -----------------------------------  Common vehicle APIs ---------------------------------------------
    def reset(self):
        self.client.call('reset')

    def ping(self):
        return self.client.call('ping')

    def getClientVersion(self):
        return 1 # sync with C++ client
    def getServerVersion(self):
        return self.client.call('getServerVersion')
    def getMinRequiredServerVersion(self):
        return 1 # sync with C++ client
    def getMinRequiredClientVersion(self):
        return self.client.call('getMinRequiredClientVersion')

    # basic flight control
    def enableApiControl(self, is_enabled, vehicle_name = ''):
        return self.client.call('enableApiControl', is_enabled, vehicle_name)
    def isApiControlEnabled(self, vehicle_name = ''):
        return self.client.call('isApiControlEnabled', vehicle_name)
    def armDisarm(self, arm, vehicle_name = ''):
        return self.client.call('armDisarm', arm, vehicle_name)
 
    def simPause(self, is_paused):
        self.client.call('simPause', is_paused)
    def simIsPause(self):
        return self.client.call("simIsPaused")
    def simContinueForTime(self, seconds):
        self.client.call('simContinueForTime', seconds)

    def getHomeGeoPoint(self, vehicle_name = ''):
        return GeoPoint.from_msgpack(self.client.call('getHomeGeoPoint', vehicle_name))

    def confirmConnection(self):
        if self.ping():
            print("Connected!")
        else:
             print("Ping returned false!")
        server_ver = self.getServerVersion()
        client_ver = self.getClientVersion()
        server_min_ver = self.getMinRequiredServerVersion()
        client_min_ver = self.getMinRequiredClientVersion()
    
        ver_info = "Client Ver:" + str(client_ver) + " (Min Req: " + str(client_min_ver) + \
              "), Server Ver:" + str(server_ver) + " (Min Req" + str(server_min_ver) + ")"

        if server_ver < server_min_ver:
            print(ver_info, file=sys.stderr)
            print("AirSim server is of older version and not supported by this client. Please upgrade!")
        elif client_ver < client_min_ver:
            print(ver_info, file=sys.stderr)
            print("AirSim client is of older version and not supported by this server. Please upgrade!")
        else:
            print(ver_info)
        print('')

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImage(self, camera_name, image_type, vehicle_name = ''):
        # todo: in future remove below, it's only for compatibility to pre v1.2
        camera_name = str(camera_name)

        # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
        result = self.client.call('simGetImage', camera_name, image_type, vehicle_name)
        if (result == "" or result == "\0"):
            return None
        return result

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the ImageType members
    def simGetImages(self, requests, vehicle_name = ''):
        responses_raw = self.client.call('simGetImages', requests, vehicle_name)
        return [ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]

    def simGetCollisionInfo(self, vehicle_name = ''):
        return CollisionInfo.from_msgpack(self.client.call('simGetCollisionInfo', vehicle_name))

    def simSetVehiclePose(self, pose, ignore_collison, vehicle_name = ''):
        self.client.call('simSetVehiclePose', pose, ignore_collison, vehicle_name)
    def simGetVehiclePose(self, vehicle_name = ''):
        pose = self.client.call('simGetVehiclePose', vehicle_name)
        return Pose.from_msgpack(pose)
    def simGetObjectPose(self, object_name):
        pose = self.client.call('simGetObjectPose', object_name)
        return Pose.from_msgpack(pose)

    def simSetSegmentationObjectID(self, mesh_name, object_id, is_name_regex = False):
        return self.client.call('simSetSegmentationObjectID', mesh_name, object_id, is_name_regex)
    def simGetSegmentationObjectID(self, mesh_name):
        return self.client.call('simGetSegmentationObjectID', mesh_name)
    def simPrintLogMessage(self, message, message_param = "", severity = 0):
        return self.client.call('simPrintLogMessage', message, message_param, severity)

    def simGetCameraInfo(self, camera_name, vehicle_name = ''):
        return CameraInfo.from_msgpack(self.client.call('simGetCameraInfo', camera_name, vehicle_name))
    def simSetCameraOrientation(self, camera_name, orientation, vehicle_name = ''):
        self.client.call('simSetCameraOrientation', camera_name, orientation, vehicle_name)

    def cancelLastTask():
        self.client.call('cancelLastTask')
    def waitOnLastTask(timeout_sec = float('nan')):
        return self.client.call('waitOnLastTask', timeout_sec)

    def simGetPosition(self):
        raise Exception("simGetPosition is deprecated. Please use simGetVehiclePose") 
    def simSetPosition(self, depricated1, depricated2):
        raise Exception("simSetPosition is deprecated. Please use simSetVehiclePose") 

# -----------------------------------  Multirotor APIs ---------------------------------------------
class MultirotorClient(VehicleClient, object):
    def __init__(self):
        super(MultirotorClient, self).__init__(ip, 41451, timeout_value = timeout)

    def takeoffAsync(self, timeout_sec = 20, vehicle_name = ''):
        return self.client.call_async('takeoff', timeout_sec, vehicle_name)  
    def landAsync(self, timeout_sec = 60, vehicle_name = ''):
        return self.client.call_async('land', timeout_sec, vehicle_name)   
    def goHomeAsync(self, timeout_sec = 3e+38, vehicle_name = ''):
        return self.client.call_async('goHome', timeout_sec, vehicle_name)

    # APIs for control
    def moveByAngleZAsync(self, pitch, roll, z, yaw, duration, vehicle_name = ''):
        return self.client.call_async('moveByAngleZ', pitch, roll, z, yaw, duration, vehicle_name)
    def moveByAngleThrottleAsync(self, pitch, roll, throttle, yaw_rate, duration, vehicle_name = ''):
        return self.client.call_async('moveByAngleThrottle', pitch, roll, throttle, yaw_rate, duration, vehicle_name)
    def moveByVelocityAsync(self, vx, vy, vz, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        return self.client.call_async('moveByVelocity', vx, vy, vz, duration, drivetrain, yaw_mode, vehicle_name)
    def moveByVelocityZAsync(self, vx, vy, z, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        return self.client.call_async('moveByVelocityZ', vx, vy, z, duration, drivetrain, yaw_mode, vehicle_name)
    def moveOnPathAsync(self, path, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), 
        lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        return self.client.call_async('moveOnPath', path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)
    def moveToPositionAsync(self, x, y, z, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), 
        lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        return self.client.call_async('moveToPosition', x, y, z, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)
    def moveToZAsync(self, z, velocity, timeout_sec = 3e+38, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        return self.client.call_async('moveToZ', z, velocity, timeout_sec, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)
    def moveByManualAsync(self, vx_max, vy_max, z_min, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        """Read current RC state and use it to control the vehicles. 

        Parameters sets up the constraints on velocity and minimum altitude while flying. If RC state is detected to violate these constraints
        then that RC state would be ignored.

        :param vx_max: max velocity allowed in x direction
        :param vy_max: max velocity allowed in y direction
        :param vz_max: max velocity allowed in z direction
        :param z_min: min z allowed for vehicle position
        :param duration: after this duration vehicle would switch back to non-manual mode
        :param drivetrain: when ForwardOnly, vehicle rotates itself so that its front is always facing the direction of travel. If MaxDegreeOfFreedom then it doesn't do that (crab-like movement)
        :param yaw_mode: Specifies if vehicle should face at given angle (is_rate=False) or should be rotating around its axis at given rate (is_rate=True)
        """
        return self.client.call_async('moveByManual', vx_max, vy_max, z_min, duration, drivetrain, yaw_mode, vehicle_name)
    def rotateToYawAsync(self, yaw, timeout_sec = 3e+38, margin = 5, vehicle_name = ''):
        return self.client.call_async('rotateToYaw', yaw, timeout_sec, margin, vehicle_name)
    def rotateByYawRateAsync(self, yaw_rate, duration, vehicle_name = ''):
        return self.client.call_async('rotateByYawRate', yaw_rate, duration, vehicle_name)
    def hoverAsync(self, vehicle_name = ''):
        return self.client.call_async('hover', vehicle_name)

    def moveByRC(self, rcdata = RCData(), vehicle_name = ''):
        return self.client.call('moveByRC', rcdata, vehicle_name)
        
    # query vehicle state
    def getMultirotorState(self, vehicle_name = ''):
        return MultirotorState.from_msgpack(self.client.call('getMultirotorState', vehicle_name))
    getMultirotorState.__annotations__ = {'return': MultirotorState}


# -----------------------------------  Car APIs ---------------------------------------------
class CarClient(VehicleClient, object):
    def __init__(self):
        if (ip == ""):
            ip = "127.0.0.1"
        super(CarClient, self).__init__(ip, 41451, timeout_value = timeout)

    def setCarControls(self, controls, vehicle_name = ''):
        self.client.call('setCarControls', controls, vehicle_name)

    def getCarState(self, vehicle_name = ''):
        state_raw = self.client.call('getCarState', vehicle_name)
        return CarState.from_msgpack(state_raw)
