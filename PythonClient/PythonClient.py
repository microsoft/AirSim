from __future__ import print_function
import msgpackrpc #pip install msgpack-rpc-python
import numpy as np #pip install numpy
import msgpack
import math
import time
import sys
import os


class MsgpackMixin:
    def to_msgpack(self, *args, **kwargs):
        return self.__dict__ #msgpack.dump(self.to_dict(*args, **kwargs))

    @classmethod
    def from_msgpack(cls, encoded):
        obj = cls()
        obj.__dict__ = {k.decode('utf-8'): v for k, v in encoded.items()}
        return obj


class AirSimImageType:    
    Scene = 0
    DepthMeters = 1
    DepthVis = 2
    DisparityNormalized = 3
    Segmentation = 4
    SurfaceNormals = 5

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

class CollisionInfo(MsgpackMixin):
    has_collided = False
    normal = Vector3r()
    impact_point = Vector3r()
    position = Vector3r()
    penetration_depth = np.float32(0)
    time_stamp = np.float32(0)

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

class AirSimClient:
        def __init__(self, ip = ""):
            if (ip == ""):
                ip = "127.0.0.1"
            self.client = msgpackrpc.Client(msgpackrpc.Address(ip, 41451), timeout = 3600)

        
        def ping(self):
            return self.client.call('ping')
        def confirmConnection(self):
            print('Waiting for connection: ', end='')
            home = self.getHomeGeoPoint()
            while ((home.latitude == 0 and home.longitude == 0 and home.altitude == 0) or
                   math.isnan(home.latitude) or  math.isnan(home.longitude) or  math.isnan(home.altitude)):
                time.sleep(1)
                home = self.getHomeGeoPoint()
                print('X', end='')
            print('')

        # basic flight control
        def enableApiControl(self, is_enabled):
            return self.client.call('enableApiControl', is_enabled)
        def isApiControlEnabled(self):
            return self.client.call('isApiControlEnabled')

        def armDisarm(self, arm):
            return self.client.call('armDisarm', arm)

        def takeoff(self, max_wait_seconds = 15):
            return self.client.call('takeoff', max_wait_seconds)
        
        def land(self, max_wait_seconds = 60):
            return self.client.call('land', max_wait_seconds)
        
        def goHome(self):
            return self.client.call('goHome')

        def hover(self):
            return self.client.call('hover')

        
        # query vehicle state
        def getPosition(self):
            return Vector3r.from_msgpack(self.client.call('getPosition'))
        def getVelocity(self):
            return Vector3r.from_msgpack(self.client.call('getVelocity'))
        def getOrientation(self):
            return Quaternionr.from_msgpack(self.client.call('getOrientation'))
        def getLandedState(self):
            return self.client.call('getLandedState')
        def getGpsLocation(self):
            return GeoPoint.from_msgpack(self.client.call('getGpsLocation'))
        def getHomeGeoPoint(self):
            return GeoPoint.from_msgpack(self.client.call('getHomeGeoPoint'))
        def getRollPitchYaw(self):
            return self.toEulerianAngle(self.getOrientation())
        def getCollisionInfo(self):
            return CollisionInfo.from_msgpack(self.client.call('getCollisionInfo'))
        #def getRCData(self):
        #    return self.client.call('getRCData')
        def timestampNow(self):
            return self.client.call('timestampNow')
        def isApiControlEnabled(self):
            return self.client.call('isApiControlEnabled')
        def isSimulationMode(self):
            return self.client.call('isSimulationMode')
        def getServerDebugInfo(self):
            return self.client.call('getServerDebugInfo')


        # APIs for control
        def moveByAngle(self, pitch, roll, z, yaw, duration):
            return self.client.call('moveByAngle', pitch, roll, z, yaw, duration)

        def moveByVelocity(self, vx, vy, vz, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode()):
            return self.client.call('moveByVelocity', vx, vy, vz, duration, drivetrain, yaw_mode)

        def moveByVelocityZ(self, vx, vy, z, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode()):
            return self.client.call('moveByVelocityZ', vx, vy, z, duration, drivetrain, yaw_mode)

        def moveOnPath(self, path, velocity, max_wait_seconds = 60, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1):
            return self.client.call('moveOnPath', path, velocity, max_wait_seconds, drivetrain, yaw_mode, lookahead, adaptive_lookahead)

        def moveToZ(self, z, velocity, max_wait_seconds = 60, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1):
            return self.client.call('moveToZ', z, velocity, max_wait_seconds, yaw_mode, lookahead, adaptive_lookahead)

        def moveToPosition(self, x, y, z, velocity, max_wait_seconds = 60, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1):
            return self.client.call('moveToPosition', x, y, z, velocity, max_wait_seconds, drivetrain, yaw_mode, lookahead, adaptive_lookahead)

        def moveByManual(self, vx_max, vy_max, z_min, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode()):
            return self.client.call('moveByManual', vx_max, vy_max, z_min, duration, drivetrain, yaw_mode)

        def rotateToYaw(self, yaw, max_wait_seconds = 60, margin = 5):
            return self.client.call('rotateToYaw', yaw, max_wait_seconds, margin)

        def rotateByYawRate(self, yaw_rate, duration):
            return self.client.call('rotateByYawRate', yaw_rate, duration)

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
            return AirSimClient.listTo2DFloatArray(response.image_data_float, response.width, response.height)

        @staticmethod
        def write_file(filename, bstr):
            with open(filename, 'wb') as afile:
                afile.write(bstr)

        def simSetPose(self, position, orientation):
            return self.client.call('simSetPose', position, orientation)

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

            file.write(bytes('PF\n', 'UTF-8') if color else bytes('Pf\n', 'UTF-8'))
            temp_str = '%d %d\n' % (image.shape[1], image.shape[0])
            file.write(bytes(temp_str, 'UTF-8'))

            endian = image.dtype.byteorder

            if endian == '<' or endian == '=' and sys.byteorder == 'little':
                scale = -scale

            temp_str = '%f\n' % scale
            file.write(bytes(temp_str, 'UTF-8'))

            image.tofile(file)
