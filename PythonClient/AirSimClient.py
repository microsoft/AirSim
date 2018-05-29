from __future__ import print_function
import msgpackrpc #install as admin: pip install msgpack-rpc-python
import numpy as np #pip install numpy
import msgpack
import math
import time
import sys
import os
import inspect
import types
import re


class MsgpackMixin:
    def __repr__(self):
        from pprint import pformat
        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)

    def to_msgpack(self, *args, **kwargs):
        return self.__dict__

    @classmethod
    def from_msgpack(cls, encoded):
        obj = cls()
        #obj.__dict__ = {k.decode('utf-8'): (from_msgpack(v.__class__, v) if hasattr(v, "__dict__") else v) for k, v in encoded.items()}
        obj.__dict__ = { k : (v if not isinstance(v, dict) else getattr(getattr(obj, k).__class__, "from_msgpack")(v)) for k, v in encoded.items()}
        #return cls(**msgpack.unpack(encoded))
        return obj


class AirSimImageType:    
    Scene = 0
    DepthPlanner = 1
    DepthPerspective = 2
    DepthVis = 3
    DisparityNormalized = 4
    Segmentation = 5
    SurfaceNormals = 6
    Infrared = 7

class DrivetrainType:
    MaxDegreeOfFreedom = 0
    ForwardOnly = 1
    
class LandedState:
    Landed = 0
    Flying = 1

class Vector3r(MsgpackMixin):
    x_val = np.float32(0)
    y_val = np.float32(0)
    z_val = np.float32(0)

    def __init__(self, x_val = np.float32(0), y_val = np.float32(0), z_val = np.float32(0)):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val


class Quaternionr(MsgpackMixin):
    w_val = np.float32(0)
    x_val = np.float32(0)
    y_val = np.float32(0)
    z_val = np.float32(0)

    def __init__(self, x_val = np.float32(0), y_val = np.float32(0), z_val = np.float32(0), w_val = np.float32(1)):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val
        self.w_val = w_val

class Pose(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()

    def __init__(self, position_val = Vector3r(), orientation_val = Quaternionr()):
        self.position = position_val
        self.orientation = orientation_val


class CollisionInfo(MsgpackMixin):
    has_collided = False
    normal = Vector3r()
    impact_point = Vector3r()
    position = Vector3r()
    penetration_depth = np.float32(0)
    time_stamp = np.float32(0)
    object_name = ""
    object_id = -1

class GeoPoint(MsgpackMixin):
    latitude = 0.0
    longitude = 0.0
    altitude = 0.0

class YawMode(MsgpackMixin):
    is_rate = True
    yaw_or_rate = 0.0
    def __init__(self, is_rate = True, yaw_or_rate = 0.0):
        self.is_rate = is_rate
        self.yaw_or_rate = yaw_or_rate

class RCData(MsgpackMixin):
    timestamp = 0
    pitch, roll, throttle, yaw = (0.0,)*4 #init 4 variable to 0.0
    switch1, switch2, switch3, switch4 = (0,)*4
    switch5, switch6, switch7, switch8 = (0,)*4
    is_initialized = False
    is_valid = False
    def __init__(self, timestamp = 0, pitch = 0.0, roll = 0.0, throttle = 0.0, yaw = 0.0, switch1 = 0,
                 switch2 = 0, switch3 = 0, switch4 = 0, switch5 = 0, switch6 = 0, switch7 = 0, switch8 = 0, is_initialized = False, is_valid = False):
        self.timestamp = timestamp
        self.pitch = pitch 
        self.roll = roll
        self.throttle = throttle 
        self.yaw = yaw 
        self.switch1 = switch1 
        self.switch2 = switch2 
        self.switch3 = switch3 
        self.switch4 = switch4 
        self.switch5 = switch5
        self.switch6 = switch6 
        self.switch7 = switch7 
        self.switch8 = switch8 
        self.is_initialized = is_initialized
        self.is_valid = is_valid

class ImageRequest(MsgpackMixin):
    camera_id = np.uint8(0)
    image_type = AirSimImageType.Scene
    pixels_as_float = False
    compress = False

    def __init__(self, camera_id, image_type, pixels_as_float = False, compress = True):
        self.camera_id = camera_id
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress


class ImageResponse(MsgpackMixin):
    image_data_uint8 = np.uint8(0)
    image_data_float = np.float32(0)
    camera_position = Vector3r()
    camera_orientation = Quaternionr()
    time_stamp = np.uint64(0)
    message = ''
    pixels_as_float = np.float32(0)
    compress = True
    width = 0
    height = 0
    image_type = AirSimImageType.Scene

class CarControls(MsgpackMixin):
    throttle = np.float32(0)
    steering = np.float32(0)
    brake = np.float32(0)
    handbrake = False
    is_manual_gear = False
    manual_gear = 0
    gear_immediate = True

    def __init__(self, throttle = 0, steering = 0, brake = 0, 
        handbrake = False, is_manual_gear = False, manual_gear = 0, gear_immediate = True):
        self.throttle = throttle
        self.steering = steering
        self.brake = brake
        self.handbrake = handbrake
        self.is_manual_gear = is_manual_gear
        self.manual_gear = manual_gear
        self.gear_immediate = gear_immediate


    def set_throttle(self, throttle_val, forward):
        if (forward):
            is_manual_gear = False
            manual_gear = 0
            throttle = abs(throttle_val)
        else:
            is_manual_gear = False
            manual_gear = -1
            throttle = - abs(throttle_val)

class KinematicsState(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()
    linear_velocity = Vector3r()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()
    angular_acceleration = Vector3r()

class CarState(MsgpackMixin):
    speed = np.float32(0)
    gear = 0
    rpm = np.float32(0)
    maxrpm = np.float32(0)
    handbrake = False
    collision = CollisionInfo();
    kinematics_true = KinematicsState()
    timestamp = np.uint64(0)

class MultirotorState(MsgpackMixin):
    collision = CollisionInfo();
    kinematics_estimated = KinematicsState()
    gps_location = GeoPoint()
    timestamp = np.uint64(0)
    landed_state = LandedState.Landed
    rc_data = RCData()

class CameraInfo(MsgpackMixin):
    pose = Pose()
    fov = -1

class AirSimClientBase:
    def __init__(self, ip, port, timeout_value = 3600):
        self.client = msgpackrpc.Client(msgpackrpc.Address(ip, port), timeout = timeout_value, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')
        
    @staticmethod
    def stringToUint8Array(bstr):
        return np.fromstring(bstr, np.uint8)
    @staticmethod
    def stringToFloatArray(bstr):
        return np.fromstring(bstr, np.float32)
    @staticmethod
    def listTo2DFloatArray(flst, width, height):
        return np.reshape(np.asarray(flst, np.float32), (height, width))
    @staticmethod
    def getPfmArray(response):
        return AirSimClientBase.listTo2DFloatArray(response.image_data_float, response.width, response.height)

    @staticmethod
    def get_public_fields(obj):
        return [attr for attr in dir(obj)
                             if not (attr.startswith("_") 
                                or inspect.isbuiltin(attr)
                                or inspect.isfunction(attr)
                                or inspect.ismethod(attr))]


    @staticmethod
    def to_dict(obj):
        return dict([attr, getattr(obj, attr)] for attr in AirSimClientBase.get_public_fields(obj))

    @staticmethod
    def to_str(obj):
        return str(AirSimClientBase.to_dict(obj))

    @staticmethod
    def write_file(filename, bstr):
        with open(filename, 'wb') as afile:
            afile.write(bstr)

    # helper method for converting getOrientation to roll/pitch/yaw
    # https:#en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    @staticmethod
    def toEulerianAngle(q):
        z = q.z_val
        y = q.y_val
        x = q.x_val
        w = q.w_val
        ysqr = y * y

        # roll (x-axis rotation)
        t0 = +2.0 * (w*x + y*z)
        t1 = +1.0 - 2.0*(x*x + ysqr)
        roll = math.atan2(t0, t1)

        # pitch (y-axis rotation)
        t2 = +2.0 * (w*y - z*x)
        if (t2 > 1.0):
            t2 = 1
        if (t2 < -1.0):
            t2 = -1.0
        pitch = math.asin(t2)

        # yaw (z-axis rotation)
        t3 = +2.0 * (w*z + x*y)
        t4 = +1.0 - 2.0 * (ysqr + z*z)
        yaw = math.atan2(t3, t4)

        return (pitch, roll, yaw)

    @staticmethod
    def toQuaternion(pitch, roll, yaw):
        t0 = math.cos(yaw * 0.5)
        t1 = math.sin(yaw * 0.5)
        t2 = math.cos(roll * 0.5)
        t3 = math.sin(roll * 0.5)
        t4 = math.cos(pitch * 0.5)
        t5 = math.sin(pitch * 0.5)

        q = Quaternionr()
        q.w_val = t0 * t2 * t4 + t1 * t3 * t5 #w
        q.x_val = t0 * t3 * t4 - t1 * t2 * t5 #x
        q.y_val = t0 * t2 * t5 + t1 * t3 * t4 #y
        q.z_val = t1 * t2 * t4 - t0 * t3 * t5 #z
        return q

    @staticmethod
    def wait_key(message = ''):
        ''' Wait for a key press on the console and return it. '''
        if message != '':
            print (message)

        result = None
        if os.name == 'nt':
            import msvcrt
            result = msvcrt.getch()
        else:
            import termios
            fd = sys.stdin.fileno()

            oldterm = termios.tcgetattr(fd)
            newattr = termios.tcgetattr(fd)
            newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
            termios.tcsetattr(fd, termios.TCSANOW, newattr)

            try:
                result = sys.stdin.read(1)
            except IOError:
                pass
            finally:
                termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)

        return result

    @staticmethod
    def read_pfm(file):
        """ Read a pfm file """
        file = open(file, 'rb')

        color = None
        width = None
        height = None
        scale = None
        endian = None

        header = file.readline().rstrip()
        header = str(bytes.decode(header, encoding='utf-8'))
        if header == 'PF':
            color = True
        elif header == 'Pf':
            color = False
        else:
            raise Exception('Not a PFM file.')

        temp_str = str(bytes.decode(file.readline(), encoding='utf-8'))
        dim_match = re.match(r'^(\d+)\s(\d+)\s$', temp_str)
        if dim_match:
            width, height = map(int, dim_match.groups())
        else:
            raise Exception('Malformed PFM header.')

        scale = float(file.readline().rstrip())
        if scale < 0: # little-endian
            endian = '<'
            scale = -scale
        else:
            endian = '>' # big-endian

        data = np.fromfile(file, endian + 'f')
        shape = (height, width, 3) if color else (height, width)

        data = np.reshape(data, shape)
        # DEY: I don't know why this was there.
        #data = np.flipud(data)
        file.close()
    
        return data, scale

    @staticmethod
    def write_pfm(file, image, scale=1):
        """ Write a pfm file """
        file = open(file, 'wb')

        color = None

        if image.dtype.name != 'float32':
            raise Exception('Image dtype must be float32.')

        image = np.flipud(image)

        if len(image.shape) == 3 and image.shape[2] == 3: # color image
            color = True
        elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1: # greyscale
            color = False
        else:
            raise Exception('Image must have H x W x 3, H x W x 1 or H x W dimensions.')

        file.write('PF\n'.encode('utf-8')  if color else 'Pf\n'.encode('utf-8'))
        temp_str = '%d %d\n' % (image.shape[1], image.shape[0])
        file.write(temp_str.encode('utf-8'))

        endian = image.dtype.byteorder

        if endian == '<' or endian == '=' and sys.byteorder == 'little':
            scale = -scale

        temp_str = '%f\n' % scale
        file.write(temp_str.encode('utf-8'))

        image.tofile(file)

    @staticmethod
    def write_png(filename, image):
        """ image must be numpy array H X W X channels
        """
        import zlib, struct

        buf = image.flatten().tobytes()
        width = image.shape[1]
        height = image.shape[0]

        # reverse the vertical line order and add null bytes at the start
        width_byte_4 = width * 4
        raw_data = b''.join(b'\x00' + buf[span:span + width_byte_4]
                            for span in range((height - 1) * width_byte_4, -1, - width_byte_4))

        def png_pack(png_tag, data):
            chunk_head = png_tag + data
            return (struct.pack("!I", len(data)) +
                    chunk_head +
                    struct.pack("!I", 0xFFFFFFFF & zlib.crc32(chunk_head)))

        png_bytes = b''.join([
            b'\x89PNG\r\n\x1a\n',
            png_pack(b'IHDR', struct.pack("!2I5B", width, height, 8, 6, 0, 0, 0)),
            png_pack(b'IDAT', zlib.compress(raw_data, 9)),
            png_pack(b'IEND', b'')])

        AirSimClientBase.write_file(filename, png_bytes)


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
    def enableApiControl(self, is_enabled):
        return self.client.call('enableApiControl', is_enabled)
    def isApiControlEnabled(self):
        return self.client.call('isApiControlEnabled')
    def armDisarm(self, arm):
        return self.client.call('armDisarm', arm)
 
    def simPause(self, is_paused):
        self.client.call('simPause', is_paused)
    def simIsPause(self):
        return self.client.call("simIsPaused")
    def simContinueForTime(self, seconds):
        self.client.call('simContinueForTime', seconds)

    def getHomeGeoPoint(self):
        return GeoPoint.from_msgpack(self.client.call('getHomeGeoPoint'))

    def confirmConnection(self):
        home = self.getHomeGeoPoint()
        while ((home.latitude == 0 and home.longitude == 0 and home.altitude == 0) or
                math.isnan(home.latitude) or  math.isnan(home.longitude) or  math.isnan(home.altitude)):
            time.sleep(0.2)
            home = self.getHomeGeoPoint()
            print('X', end='')
        print(" Connected!")
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
    # image_type uses one of the AirSimImageType members
    def simGetImage(self, camera_id, image_type):
        # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
        result = self.client.call('simGetImage', camera_id, image_type)
        if (result == "" or result == "\0"):
            return None
        return result

    # camera control
    # simGetImage returns compressed png in array of bytes
    # image_type uses one of the AirSimImageType members
    def simGetImages(self, requests):
        responses_raw = self.client.call('simGetImages', requests)
        return [ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]

    def simGetCollisionInfo(self):
        return CollisionInfo.from_msgpack(self.client.call('simGetCollisionInfo'))

    def simSetVehiclePose(self, pose, ignore_collison):
        self.client.call('simSetVehiclePose', pose, ignore_collison)
    def simGetVehiclePose(self):
        pose = self.client.call('simGetVehiclePose')
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

    def simGetCameraInfo(self, camera_id):
        return CameraInfo.from_msgpack(self.client.call('simGetCameraInfo', camera_id))
    def simSetCameraOrientation(self, camera_id, orientation):
        self.client.call('simSetCameraOrientation', camera_id, orientation)

    def cancelLastTask():
        self.client.call('cancelLastTask')
    def waitOnLastTask(timeout_sec = float('nan')):
        return self.client.call('waitOnLastTask', timeout_sec)

# -----------------------------------  Multirotor APIs ---------------------------------------------
class MultirotorClient(AirSimClientBase, object):
    def __init__(self, ip = "", timeout = 3600):
        if (ip == ""):
            ip = "127.0.0.1"
        super(MultirotorClient, self).__init__(ip, 41451, timeout_value = timeout)

    def takeoffAsync(self, timeout_sec = 20):
        return self.client.call_async('takeoff', timeout_sec)  
    def landAsync(self, timeout_sec = 60):
        return self.client.call_async('land', timeout_sec)   
    def goHomeAsync(self, timeout_sec = 3e+38):
        return self.client.call_async('goHome', timeout_sec)

    # APIs for control
    def moveByAngleZAsync(self, pitch, roll, z, yaw, duration):
        return self.client.call_async('moveByAngleZ', pitch, roll, z, yaw, duration)
    def moveByAngleThrottleAsync(self, pitch, roll, throttle, yaw_rate, duration):
        return self.client.call_async('moveByAngleThrottle', pitch, roll, throttle, yaw_rate, duration)
    def moveByVelocityAsync(self, vx, vy, vz, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode()):
        return self.client.call_async('moveByVelocity', vx, vy, vz, duration, drivetrain, yaw_mode)
    def moveByVelocityZAsync(self, vx, vy, z, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode()):
        return self.client.call_async('moveByVelocityZ', vx, vy, z, duration, drivetrain, yaw_mode)
    def moveOnPathAsync(self, path, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1):
        return self.client.call_async('moveOnPath', path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead)
    def moveToPositionAsync(self, x, y, z, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1):
        return self.client.call_async('moveToPosition', x, y, z, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead)
    def moveToZAsync(self, z, velocity, timeout_sec = 3e+38, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1):
        return self.client.call_async('moveToZ', z, velocity, timeout_sec, yaw_mode, lookahead, adaptive_lookahead)
    def moveByManualAsync(self, vx_max, vy_max, z_min, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode()):
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
        return self.client.call_async('moveByManual', vx_max, vy_max, z_min, duration, drivetrain, yaw_mode)
    def rotateToYawAsync(self, yaw, timeout_sec = 3e+38, margin = 5):
        return self.client.call_async('rotateToYaw', yaw, timeout_sec, margin)
    def rotateByYawRateAsync(self, yaw_rate, duration):
        return self.client.call_async('rotateByYawRate', yaw_rate, duration)
    def hoverAsync(self):
        return self.client.call_async('hover')

    def moveByRC(self, rcdata = RCData()):
        return self.client.call('moveByRC', rcdata)
        
    # query vehicle state
    def getMultirotorState(self):
        return MultirotorState.from_msgpack(self.client.call('getMultirotorState'))

    def getPitchRollYawAsync(self):
        return self.toEulerianAngle(self.getOrientation())
    getMultirotorState.__annotations__ = {'return': MultirotorState}

# -----------------------------------  Car APIs ---------------------------------------------
class CarClient(AirSimClientBase, object):
    def __init__(self, ip = "", timeout = 3600):
        if (ip == ""):
            ip = "127.0.0.1"
        super(CarClient, self).__init__(ip, 41451, timeout_value = timeout)

    def setCarControls(self, controls):
        self.client.call('setCarControls', controls)

    def getCarState(self):
        state_raw = self.client.call('getCarState')
        return CarState.from_msgpack(state_raw)
