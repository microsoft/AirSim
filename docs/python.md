# Using Python with AirSim

You can call AirSim APIs from Python to control the vehicle and get vehicle state as well as images back.  

We recommend using Anaconda with Python 3.5 or later versions however some code may also work with Python 2.7 ([help us](contributing.md) improve compatibility!).

## Quick Start
First install this package:

```
pip install msgpack-rpc-python
```

You can either get AirSim binaries from [releases](https://github.com/Microsoft/AirSim/releases) or compile from the source ([Windows](https://github.com/Microsoft/AirSim/blob/master/docs/build_windows.md), [Linux](https://github.com/Microsoft/AirSim/blob/master/docs/build_linux.md)). Once you can run AirSim, choose Car as vehicle and then navigate to `PythonClient\car\` folder and run:

```
python hello_car.py
```

If you are using Visual Studio 2017 then just open AirSim.sln, set PythonClient as startup project and choose `car\hello_car.py` as your startup script.

## Installing AirSim Package
You can also install `airsim` package simply by,

```
pip install airsim
```

You can find source code and samples for this package in `PythonClient` folder in your repo.

Note that AirSim is still under heavy development which means you might frequently need to update the package to use new APIs.

## Multirotor Example

The [hello_drone.py](../PythonClient/multirotor/hello_drone.py) example shows how to use APIs to control multirotor and get state as well as images using Python.

```
import airsim

# connect to the AirSim simulator 
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
client.moveToPositionAsync(-10, 10, -10, 5).join()

# take images
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.AirSimImageType.DepthVis), 
    airsim.ImageRequest("1", airsim.AirSimImageType.DepthPlanner, True)])
print('Retrieved images: %d', len(responses))

for response in responses:
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath('/temp/py1.pfm'), airsim.getPfmArray(response))
    else:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath('/temp/py1.png'), response.image_data_uint8)
```

## Car

The [hello_car.py](../PythonClient/car/hello_car.py) example shows how to use APIs to control car and get state as well as images using Python.

```
import airsim

import time

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

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
        airsim.ImageRequest(0, airsim.AirSimImageType.DepthVis),
        airsim.ImageRequest(1, airsim.AirSimImageType.DepthPlanner, True)]) 
    print('Retrieved images: %d', len(responses))

    for response in responses:
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath('c:/temp/py1.pfm'), airsim.getPfmArray(response))
        else:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath('c:/temp/py1.png'), response.image_data_uint8)
```

## Other examples
The [PythonClient folder](../PythonClient] has few other ready to run examples:

- Flying a path: shows how Chris Lovett created the [moveOnPath demo](https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo).
Make sure your drone is initially located at the start position x=310.0 cm, y=11200.0 cm, z=235.0 cm of the Modular Neighborhood map.
- Camera: shows capturing the Depth camera view from AirSim and displaying it in an OpenCV window.

## Notes on APIs

### Image APIs
- The API `simGetImage` returns `binary string literal` which means you can simply dump it in binary file to create a .png file. However if you want to process it in any other way than you can handy function `airsim.string_to_uint8_array`. This converts binary string literal to NumPy uint8 array.

- The API `simGetImages` can accept request for multiple image types from any cameras in single call. You can specify if image is png compressed, RGB uncompressed or float array. For png compressed images, you get `binary string literal`. For float array you get Python list of float64. You can convert this float array to NumPy 2D array using
    ```
    airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height)
    ```
    You can also save float array to .pfm file (Portable Float Map format) using `airsim.write_pfm()` function.

## FAQ

#### Do I need any thing else on Windows?
VS2017 should have everything if you installed VC++ and Python. To use APIs you will need Python 3.5 or later (install it using Anaconda).

#### Which version of Python should I use?
We recommend [Anaconda](https://www.anaconda.com/download/) to get Python tools and libraries. Our code is tested with Python 3.5.3 :: Anaconda 4.4.0. This is important because older version have been known to have [problems](https://stackoverflow.com/a/45934992/207661).

#### I get error on `import cv2`
You can install OpenCV using:
```
conda install opencv
pip install opencv-python
```
