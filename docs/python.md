## Using Python with AirSim

You can call AirSim from Python to control the drone and get images back.  

First install the following Python package:

````
pip install mprpc
````

Now you can run the samples in the Python folder.  For example `path.py` runs my [moveOnPath demo](https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo)
 and `camera.py` captures the depth camera and presents it in an opencv window.

## Windows

On windows you may need to install the Microsoft Visual C++ 9.0 from [http://aka.ms/vcpython27](http://aka.ms/vcpython27).

