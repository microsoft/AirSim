# airsim_ros_client

## Overview

This repository is meant to integrate ROS and AirSim plugin using the python APIs available for the simulator.

The `airsim_ros_client` package has been tested under ROS Kinetic and Ubuntu 16.04LTS. The source code is released under [MIT Licence](LICENSE).

## Installation

#### Dependencies

Install the python depenencies:
```bash
# AirSim APIs
pip install airsim
```

#### Building
* To build from source, clone the latest version from this repository into your catkin workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Mayankm96/airsim_ros_wrapper.git
```
* To compile the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

#### Running AirSim in Unreal Engine

Before running the nodes in the package, you need to run Airsim plugin in the Unreal Engine. In case you are unfamiliar on how to do so, refer to the tutorials available [here](https://github.com/Microsoft/AirSim#tutorials).

A sample `settings.json` file used to run the simulator with the ROS package is available [here](docs/settings.json). Copy it to the `~/Documents/AirSim` directory to use the package without any further modifications.

## Usage

### Running the `tf` publisher of drone model (DJI M100)

To use the [`urdf`](urdf) model of the drone used in AirSim simulator, then run:
```
roslaunch airsim_ros_wrapper publish_tf.launch
```

__NOTE:__ In the modified blueprint of the drone for UE4, all cameras are downward-facing.

### Running image publisher

Change the IP and Port configurations in [`pubImages.launch`](launch/pubImages.launch)  to match the settings in which Airsim is running. Then:
```
roslaunch airsim_ros_wrapper pubImages.launch
```

## Nodes

### airsim_img_publisher

This is a client node at ([`img_publisher.py`](scripts/img_publisher.py)) interfaces with the AirSim plugin to retrieve the drone's pose and camera images **(rgb, depth)**.

#### Published Topics

* **`/airsim/rgb/image_raw`** ([sensor_msgs/Image])

	The rgb camera images in `rgba8` encoding.

* **`/airsim/depth`** ([sensor_msgs/Image])

	The depth camera images in `32FC1` encoding.

* **`/airsim/camera_info`** ([sensor_msgs/CameraInfo])

  The rgb camera paramters.

* **`/airsim/depth/camera_info`** ([sensor_msgs/CameraInfo])

  The depth camera paramters.

* **`/airsim/pose`** ([geometry_msgs/PoseStamped])

	The position/orientation of the quadcoper (`base_link`)

* **`/odom`** ([nav_msgs/Odometry])

	The odometry of the quadcoper (`base_link`) in the `world` frame

* **`/tf`**

  tf tree with the origin (`world`), the position/orientation of the quadcoper (`base_link`)


#### Parameters
* **Camera parameters:** `Fx`, `Fy`, `cx`, `cz`, `width`, `height`
* **Publishing frequency:** `loop_rate`

### airsim_follow_trajectory

This is a client node at ([`follow_trajectory.py`](scripts/follow_trajectory.py)) interfaces with the AirSim plugin to follow a trajectory.

#### Subscribed Topics

* **`/trajectory/spline_marker_array`** ([visualization_msgs/MarkerArray])

	An array of waypoints to follow.

#### Parameters
* **Velocity:** `velocity`

[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
[geometry_msgs/PoseStamped]: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
[nav_msgs/Odometry]: http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
[visualization_msgs/MarkerArray]: http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html/Odometry.html
