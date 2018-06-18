# Python API Examples for AirSim

This folder contains Python package and examples for [AirSim](https://github.com/microsoft/airsim).

## Pre-requisites
Please make sure you have installed Python package for msgpack (this needs administrator/sudo prompt):
```
pip install msgpack-rpc-python
```

Some examples requires opencv. You can install required packages by running `install_packages.bat`.

## How to Run
Try out `car/hello_car.py` or `multirotor\hello_drone.py`

 in `airsim` folder exposing the ApiSim APIs for Python. 
Other folders contains example code for how to use APIs.

More information on AirSim Python APIs can be found at:
https://github.com/Microsoft/AirSim/blob/master/docs/python.md

Note: Do not run `setup.py` manually. It is meant for creating python package.