## Using Python with AirSim

You can call AirSim from Python to control the drone and get images back.  

First install the following Python package:

````
pip install msgpack-rpc-python
````

Now you can run the samples in the PythonClient folder.  


### Takeoff 

For example [takeoff.py](../PythonClient/takeoff.py) which
shows how to do a simple takeoff command and hover once it reaches the takeoff altitude.

````
from PythonClient import *
import sys
import time
import msgpackrpc
import math

# connect to the AirSim simulator 
client = AirSimClient('127.0.0.1')

print("Waiting for home GPS location to be set...")
home = client.getHomePoint()
while ((home[0] == 0 and home[1] == 0 and home[2] == 0) or
       math.isnan(home[0]) or  math.isnan(home[1]) or  math.isnan(home[2])):
    time.sleep(1)
    home = client.getHomePoint()

print("Home lat=%g, lon=%g, alt=%g" % tuple(home))

if (not client.arm()):
    print("failed to arm the drone")
    sys.exit(1);

if (client.getLandedState() == LandedState.Landed):
    print("Taking off...")
    if (not client.takeoff(60)):
        print("failed to reach takeoff altitude after 60 seconds")
        sys.exit(1);
    print("Should now be flying...")
else:
    print("it appears the drone is already flying")

client.hover();

````


### Flying a path 
Now that the drone is flying you can send some other flight commands.
See [path.py](../PythonClient/path.py) which shows how I
 created the [moveOnPath demo](https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo).
If your drone is located at the start position x=310.0 cm, y=11200.0 cm, z=235.0 cm of the Modular Neighbohood map
then the following will make the drone fly along the streets.

````
from PythonClient import *
import sys
import time

client = AirSimClient('127.0.0.1')

# AirSim uses NED coordinates so negative axis is up.
# z of -5 is 5 meters above the original launch point.
z = -5

# this method is async and we are not waiting for the result since we are passing max_wait_seconds=0.
print("client.moveOnPath to fly fast path along the streets")
result = client.moveOnPath([(0,-253,z),(125,-253,z),(125,0,z),(0,0,z)], 15, 0, DrivetrainType.ForwardOnly, YawMode(False,0), 20, 1)

````

The drone should now be flying fast along the specified path.

### Camera

While the drone is flying you might want to capture some camera images.
See  [camera.py](../PythonClient/camera.py) in the PythonClient folder.
This program is capturing the Depth camera view from AirSim and displaying it in an OpenCV window.

````
# use open cv to show new images from AirSim 

from PythonClient import *
import cv2
import time
import sys

client = AirSimClient('127.0.0.1')

# get depth image
result = client.setImageTypeForCamera(0, AirSimImageType.Depth)

time.sleep(1) # give it time to render
help = False

while True:
    # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
    result = client.simGetImage(0, AirSimImageType.Depth)
    if (result == "\0"):
        if (not help):
            help = True
            print("Please press '1' in the AirSim view to enable the Depth camera view")
    else:
        rawImage = np.fromstring(result, np.int8)
        png = cv2.imdecode(rawImage, cv2.IMREAD_UNCHANGED)
        cv2.imshow("Traffic", png)

    key = cv2.waitKey(1) & 0xFF;
    if (key == 27 or key == ord('q') or key == ord('x')):
        break;

````

## Windows

On windows you may need to install the Microsoft Visual C++ 9.0 from [http://aka.ms/vcpython27](http://aka.ms/vcpython27).

