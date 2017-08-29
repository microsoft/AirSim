# Using Python with AirSim

You can call AirSim APIs from Python to control the drone and get vehicle state and images back.  

First install the following Python package:

````
pip install msgpack-rpc-python
````

If you are using Visual Studio then click on "Python Environments" under your Python project. Then right click on environment and choose "Install Python Package...".

Now you can run the samples in the [PythonClient](../PythonClient) folder.  


## Getting Started

The [hello_drone.py](../PythonClient/hello_drone.py) example shows basic operations using Python.

```
from PythonClient import *

# connect to the AirSim simulator 
client = AirSimClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

AirSimClient.wait_key('Press any key to takeoff')
client.takeoff()

AirSimClient.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPosition(-10, 10, -10, 5)

AirSimClient.wait_key('Press any key to take images')
responses = client.simGetImages([
    ImageRequest(0, AirSimImageType.DepthVis), 
    ImageRequest(1, AirSimImageType.DepthMeters, True)])
print('Retrieved images: %d', len(responses))

for response in responses:
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        AirSimClient.write_pfm(os.path.normpath('/temp/py1.pfm'), AirSimClient.getPfmArray(response))
    else:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        AirSimClient.write_file(os.path.normpath('/temp/py1.png'), response.image_data_uint8)
```

## Other examples
The [PythonClient folder](../PythonClient] has few other ready to run examples:

- Flying a path: shows how Chris Lovett created the [moveOnPath demo](https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo).
Make sure your drone is initially located at the start position x=310.0 cm, y=11200.0 cm, z=235.0 cm of the Modular Neighbohood map.
- Camera: shows capturing the Depth camera view from AirSim and displaying it in an OpenCV window.

## Notes on APIs

### Image APIs
- The API `simGetImage` returns `binary string literal` which means you can simply dump it in binary file to create a .png file. However if you want to process it in any other way than you can handy function `AirSimClient.stringToUint8Array`. This converts binary string literal to NumPy uint8 array.

- The API `simGetImages` can accept request for multiple image types from any cameras in single call. You can specify if image is png compressed, RGB uncompressed or float array. For png compressed images, you get `binary string literal`. For float array you get Python list of float64. You can convert this float array to NumPy 2D array using
    ```
    AirSimClient.listTo2DFloatArray(response.image_data_float, response.width, response.height)
    ```
    You can also save float array to .pfm file (Portable Float Map format) using `AirSimClient.write_pfm()` function.

## FAQ

#### Do I need any thing else on Windows?
VS2015 Upate 3 should have everything if you installed VC++ and Python. Otherwise you may need to install the Microsoft Visual C++ 9.0 from [http://aka.ms/vcpython27](http://aka.ms/vcpython27).

#### I get error on `import cv2`
We recommand [Anaconda](https://www.anaconda.com/download/) to get Python tools and libraries. Our code is tested with Python 3.5.3 :: Anaconda 4.4.0. This is important because older version have been known to have [problems](https://stackoverflow.com/a/45934992/207661). You can install OpenCV using `pip install opencv-python`.

