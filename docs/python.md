# Using Python with AirSim

You can call AirSim APIs from Python to control the vehicle and get vehicle state as well as images back.  

Python 3.x is required for running sample code however AirSimClient package can be used from Python 2.7 as well.

First install the following Python package:

````
pip install msgpack-rpc-python
pip install numpy
````

If you are using Visual Studio then click on "Python Environments" under your Python project. Then right click on environment and choose "Install Python Package...". Choose pip and then type the names of above packages.

Now you can run the samples in the [PythonClient](../PythonClient) folder.  


## Getting Started with Multirotors

The [hello_drone.py](../PythonClient/hello_drone.py) example shows how to use APIs to control multirotor and get state as well as images using Python.

```
from AirSimClient import *

# connect to the AirSim simulator 
client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

MultirotorClient.wait_key('Press any key to takeoff')
client.takeoff()

MultirotorClient.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPosition(-10, 10, -10, 5)

MultirotorClient.wait_key('Press any key to take images')
responses = client.simGetImages([
    ImageRequest(0, AirSimImageType.DepthVis), 
    ImageRequest(1, AirSimImageType.DepthPlanner, True)])
print('Retrieved images: %d', len(responses))

for response in responses:
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        MultirotorClient.write_pfm(os.path.normpath('/temp/py1.pfm'), MultirotorClient.getPfmArray(response))
    else:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        MultirotorClient.write_file(os.path.normpath('/temp/py1.png'), response.image_data_uint8)
```

## Getting Started with Cars

The [hello_car.py](../PythonClient/hello_car.py) example shows how to use APIs to control car and get state as well as images using Python.

```
from AirSimClient import *

# connect to the AirSim simulator 
client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = CarControls()

while True:
    # get state of the car
    car_state = client.getCarState()
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))

    # set the controls for car
    car_controls.throttle = 1
    car_controls.steering = 1
    client.setCarControls(car_controls)

    # let car drive a bit
    time.sleep(1)

    # get camera images from the car
    responses = client.simGetImages([
        ImageRequest(0, AirSimImageType.DepthVis),
        ImageRequest(1, AirSimImageType.DepthPlanner, True)]) 
    print('Retrieved images: %d', len(responses))

    for response in responses:
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            CarClient.write_pfm(os.path.normpath('c:/temp/py1.pfm'), CarClient.getPfmArray(response))
        else:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            CarClient.write_file(os.path.normpath('c:/temp/py1.png'), response.image_data_uint8)
```

## Other examples
The [PythonClient folder](../PythonClient] has few other ready to run examples:

- Flying a path: shows how Chris Lovett created the [moveOnPath demo](https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo).
Make sure your drone is initially located at the start position x=310.0 cm, y=11200.0 cm, z=235.0 cm of the Modular Neighbohood map.
- Camera: shows capturing the Depth camera view from AirSim and displaying it in an OpenCV window.

## Notes on APIs

### Image APIs
- The API `simGetImage` returns `binary string literal` which means you can simply dump it in binary file to create a .png file. However if you want to process it in any other way than you can handy function `AirSimClientBase.stringToUint8Array`. This converts binary string literal to NumPy uint8 array.

- The API `simGetImages` can accept request for multiple image types from any cameras in single call. You can specify if image is png compressed, RGB uncompressed or float array. For png compressed images, you get `binary string literal`. For float array you get Python list of float64. You can convert this float array to NumPy 2D array using
    ```
    AirSimClientBase.listTo2DFloatArray(response.image_data_float, response.width, response.height)
    ```
    You can also save float array to .pfm file (Portable Float Map format) using `AirSimClientBase.write_pfm()` function.

## FAQ

#### Do I need any thing else on Windows?
VS2017 should have everything if you installed VC++. To use APIs you will need Python 3.5 (install using Anaconda).

#### I get error on `import cv2`
We recommand [Anaconda](https://www.anaconda.com/download/) to get Python tools and libraries. Our code is tested with Python 3.5.3 :: Anaconda 4.4.0. This is important because older version have been known to have [problems](https://stackoverflow.com/a/45934992/207661). You can install OpenCV using `pip install opencv-python`.

