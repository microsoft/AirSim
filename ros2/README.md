# AirSim ROS 2 Node

This fork of Microsoft's AirSim project contains a few changes to the ROS 2 node:
- Custom Dockerfile and build/run script.
- Fixes bug with some image types being published that are incompatible with rqt.
- Adds gimbal state topics publishing the vehicle relative orientation of the gimbal.
- Uses camera settings when updating the gimbal orientation (on subscription) to maintain the correct position according to the settings JSON.

## Running
To run:
```
./run.sh
```

## Building
To build before running:
```
./run.sh --build
```

Alternatively:
```
./run.sh -b
```
