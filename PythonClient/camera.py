# use open cv to show new images from AirSim 

from PythonClient import *
import cv2
import time
import math

# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def toEulerianAngle(q):
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


client = AirSimClient('127.0.0.1')

# you must first press "1" in the AirSim view to turn on the depth capture

# get depth image
client.setImageTypeForCamera(0, AirSimImageType.Depth)
time.sleep(1) # give it time to render
yaw = 0
pi = math.radians(90)

while True:
    # this will return png width= 256, height= 144
    rawImage = client.getImageForCamera(0, AirSimImageType.Depth)       
    png = cv2.imdecode(rawImage, cv2.IMREAD_UNCHANGED)
    gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)

    # slice the image so we only check what we are headed into (and not what is down on the ground below us).

    top = np.vsplit(gray, 2)[0]

    # now look at 4 horizontal bands (far left, left, right, far right) and see which is most open.
    bands = np.hsplit(top, [50,100,150,200]);
    means = [np.mean(x) for x in bands]
    min = np.argmin(means)

    q = client.getOrientation()
    pitch, roll, yaw = toEulerianAngle(q)

    # we have a 90 degree field of view (pi/2), we've sliced that into 5 chunks, each chunk then represents
    # an angular delta of the following pi/10.
    change = 0
    if (min == 0):
        change = -2 * pi / 10
    elif (min == 1):
        change = -pi / 10
    elif (min == 2):
        change = 0 # center strip, go straight
    elif (min == 3):
        change = pi / 10
    else:
        change = 2*pi/10
    
    degrees = (yaw + change) * 180 / pi
    print min, degrees
    #client.moveByVelocityZ(1,0,-5, 20, DrivetrainType.ForwardOnly, YawMode(False, degrees))

    cv2.imshow("Top", top)

    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        break;
