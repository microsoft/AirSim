import numpy as np #pip install numpy
import math
import time
import sys
import os
import inspect
import types
import re
import logging

from .types import *


def string_to_uint8_array(bstr):
    return np.fromstring(bstr, np.uint8)
    
def string_to_float_array(bstr):
    return np.fromstring(bstr, np.float32)
    
def list_to_2d_float_array(flst, width, height):
    return np.reshape(np.asarray(flst, np.float32), (height, width))
    
def get_pfm_array(response):
    return list_to_2d_float_array(response.image_data_float, response.width, response.height)

    
def get_public_fields(obj):
    return [attr for attr in dir(obj)
                            if not (attr.startswith("_") 
                            or inspect.isbuiltin(attr)
                            or inspect.isfunction(attr)
                            or inspect.ismethod(attr))]


    
def to_dict(obj):
    return dict([attr, getattr(obj, attr)] for attr in get_public_fields(obj))

    
def to_str(obj):
    return str(to_dict(obj))

    
def _as_bytes_like_for_file(bstr):
    if bstr is None:
        return None
    if isinstance(bstr, memoryview):
        return bstr.tobytes()
    if isinstance(bstr, bytearray):
        return bytes(bstr)
    if isinstance(bstr, bytes):
        return bstr
    if isinstance(bstr, str):
        if bstr == "" or bstr == "\0":
            return None
        return bstr.encode("latin-1", errors="ignore")
    try:
        return bytes(bstr)
    except Exception:
        return None


def write_file(filename, bstr):
    """
    Write binary data to file.
    Used for writing compressed PNG images.

    No-op when bstr is None or empty so empty simGetImage responses do not
    create zero-byte placeholders or raise TypeError on None.
    """
    data = _as_bytes_like_for_file(bstr)
    if not data or data == b"\0":
        return
    parent = os.path.dirname(filename)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(filename, 'wb') as afile:
        afile.write(data)

# helper method for converting getOrientation to roll/pitch/yaw
# https:#en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    
def to_eularian_angles(q):
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

    
def to_quaternion(pitch, roll, yaw):
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

    
def read_pfm(file):
    """ Read a pfm file """
    with open(file, 'rb') as file:
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
        if scale < 0:  # little-endian
            endian = '<'
            scale = -scale
        else:
            endian = '>'  # big-endian

        data = np.fromfile(file, endian + 'f')
        shape = (height, width, 3) if color else (height, width)

        data = np.reshape(data, shape)

        return data, scale

    
def write_pfm(file, image, scale=1):
    """ Write a pfm file """
    color = None

    if image.dtype.name != 'float32':
        raise Exception('Image dtype must be float32.')

    if len(image.shape) == 3 and image.shape[2] == 3:  # color image
        color = True
    elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1:  # grayscale
        color = False
    else:
        raise Exception('Image must have H x W x 3, H x W x 1 or H x W dimensions.')

    with open(file, 'wb') as file:
        file.write('PF\n'.encode('utf-8') if color else 'Pf\n'.encode('utf-8'))
        temp_str = '%d %d\n' % (image.shape[1], image.shape[0])
        file.write(temp_str.encode('utf-8'))

        endian = image.dtype.byteorder

        if endian == '<' or endian == '=' and sys.byteorder == 'little':
            scale = -scale

        temp_str = '%f\n' % scale
        file.write(temp_str.encode('utf-8'))

        image.tofile(file)

    
def write_png(filename, image):
    """ image must be numpy array H X W X channels
    """
    import cv2      # pip install opencv-python

    ret = cv2.imwrite(filename, image)
    if not ret:
        logging.error(f"Writing PNG file {filename} failed")
