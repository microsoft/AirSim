# airsim_ros_pkgs

A ROS wrapper over the AirSim C++ client library. 

##  Setup 
### Ubuntu 16.04
- Install ROS kinetic
- Deps:
  - `$ sudo apt-get install ros-kinetic-mavros-msgs`

### Ubuntu 18.04
- Install ROS melodic
- Deps:
  - `$ sudo apt-get install ros-melodic-mavros-msgs`

### Windows Subsytem for Linux on Windows 10
- WSL setup:
  * Get [Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/install-win10)
  * Get [Ubuntu 16.04](https://www.microsoft.com/en-us/p/ubuntu-1604-lts/9pjn388hp8c9?activetab=pivot:overviewtab) or [Ubuntu 18.04](https://www.microsoft.com/en-us/p/ubuntu-1804-lts/9n9tngvndl3q?activetab=pivot%3Aoverviewtab)  
  * Go to Ubuntu 16 / 18 instructions!


- Setup for X apps (like RViz, rqt_image_view, terminator) in Windows + WSL
  * Install [Xming X Server](https://sourceforge.net/projects/xming/). 
  * Find and run `XLaunch` from the Windows start menu.   
  Select `Multiple Windows` in first popup, `Start no client` in second popup, **only** `Clipboard` in third popup. Do **not** select `Native Opengl`.  
  * Open Ubuntu 16.04 / 18.04 session by typing `Ubuntu 16.04`  / `Ubuntu 18.04` in Windows start menu.  
  * Recommended: Install [terminator](http://www.ubuntugeek.com/terminator-multiple-gnome-terminals-in-one-window.html) : `$ sudo apt-get install terminator.` 
    - You can open terminator in a new window by entering `$ DISPLAY=:0 terminator -u`. 

##  Build
- Build AirSim 
```
git clone https://github.com/Microsoft/AirSim.git;
export AIRSIM_ROOT=$(pwd)/AirSim; # this environment variable is used by ROS' CMakeLists
cd AirSim;
./setup.sh;
./build.sh;
```
- Build ROS package

```
cd $(AIRSIM_ROOT)/ros;
catkin build
```

## Running
```
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch
roslaunch airsim_ros_pkgs airsim_rviz.launch
```

# Using AirSim ROS wrapper
The ROS wrapper is composed of two ROS nodes - the first is a wrapper over AirSim's multirotor C++ client library, and the second is a simple PD position controller.    
Let's look at the ROS API for both nodes: 

### AirSim ROS Wrapper Node
#### Publishers:
- `/airsim_node/home_geo_point` [airsim_ros_pkgs/GPSYaw](msg/GPSYaw.msg)   
GPS coordinates corresponding to home/spawn point of the drone. These are set in the airsim's settings.json file. Please see here for `settings.json`'s [documentation](https://microsoft.github.io/AirSim/docs/settings/). 

The defaults are:
```
 "OriginGeopoint": {
    "Latitude": 47.641468,
    "Longitude": -122.140165,
    "Altitude": 122
  }
```
  
- `/airsim_node/global_gps` [sensor_msgs/NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)   
This the current GPS coordinates of the drone in airsim. 

- `/airsim_node/odom_local_ned` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)   
Odometry in NED frame wrt take-off point 

- `/airsim_node/vehicle_state` [mavros_msgs/State](https://docs.ros.org/api/mavros_msgs/html/msg/State.html)   
  Currently, the drone is always `armed`. Hence, there is only one state. 

- `/airsim_node/imu_ground_truth` [sensor_msgs/Imu](https://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)   
  Not published yet
 
- `/front/left/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

- `/front/left/image_raw` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)   
  RGB image corresponding to front stereo pair's left camera.

- `/front/right/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

- `/front/right/image_raw` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)   
  RGB image corresponding to front stereo pair's left camera.

- `/front/left/depth_planar` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)   
  Ground truth depth from left camera's focal plane from AirSim. 

- `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)


#### Subscribers: 
- `/vel_cmd_body_frame` [airsim_ros_pkgs/VelCmd](msg/VelCmd.msg)    
  Ignore `vehicle_name` field, leave it to blank. We will use `vehicle_name` in future for multiple drones.

- `/vel_cmd_world_frame` [airsim_ros_pkgs/VelCmd](msg/VelCmd.msg)    
  Ignore `vehicle_name` field, leave it to blank. We will use `vehicle_name` in future for multiple drones.

- `/gimbal_angle_euler_cmd` [airsim_ros_pkgs/GimbalAngleEulerCmd](msg/GimbalAngleEulerCmd.msg)   
  Gimbal set point in euler angles.    
  Use `front_center`, `front_right`, or `front_left` as `camera_name` parameter in the message field.    
  Ignore `vehicle_name`.

- `/gimbal_angle_quat_cmd` [airsim_ros_pkgs/GimbalAngleQuatCmd](msg/GimbalAngleQuatCmd.msg)   
  Gimbal set point in quaternion.    
  Use `front_center`, `front_right`, or `front_left` as `camera_name` parameter in the message field.    
  Ignore `vehicle_name`.

#### Services:
- `/airsim_node/land` [std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)

- `/airsim_node/reset` [std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)

- `/airsim_node/takeoff` [std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)

#### Parameters:
- `/airsim_node/front_left_calib_file` [string]   
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`   
  Default: `$(airsim_ros_pkgs)/calib/front_left_376x672.yaml`. 

- `/airsim_node/front_right_calib_file` [string]    
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`   
  Default: `airsim_ros_pkgs/calib/front_right_376x672.yaml`

- `/airsim_node/update_airsim_control_every_n_sec` [double]   
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`   
  Default: 0.01 seconds.    
  Timer callback frequency for updating drone odom and state from airsim, and sending in control commands.    
  The current RPClib interface to unreal engine maxes out at 50 Hz.   
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter. 

- `/airsim_node/update_airsim_img_response_every_n_sec` [double]   
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch`   
  Default: 0.01 seconds.    
  Timer callback frequency for receiving images from all cameras in airsim.    
  The speed will depend on number of images requested and their resolution.   
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter. 

### Simple PID Position Controller Node 

#### Parameters:
- PD controller parameters:
  * `/pid_position_node/kd_x` [double],   
    `/pid_position_node/kp_y` [double],   
    `/pid_position_node/kp_z` [double],   
    `/pid_position_node/kp_yaw` [double]   
    Proportional gains

  * `/pid_position_node/kd_x` [double],   
    `/pid_position_node/kd_y` [double],   
    `/pid_position_node/kd_z` [double],   
    `/pid_position_node/kd_yaw` [double]   
    Derivative gains

  * `/pid_position_node/reached_thresh_xyz` [double]   
    Threshold euler distance (meters) from current position to setpoint position 

  * `/pid_position_node/reached_yaw_degrees` [double]   
    Threshold yaw distance (degrees) from current position to setpoint position 

- `/pid_position_node/update_control_every_n_sec` [double]   
  Default: 0.01 seconds

#### Services:
- `/airsim_node/gps_goal` [Request: [msgs/airsim_ros_pkgs/GPSYaw](msgs/airsim_ros_pkgs/GPSYaw)]   
  Target gps position + yaw.   
  In **absolute** altitude. 

- `/airsim_node/local_position_goal` [Request: [msgs/airsim_ros_pkgs/XYZYaw](msgs/airsim_ros_pkgs/XYZYaw)   
  Target local position + yaw in NED frame.   

#### Subscribers:
- `/airsim_node/home_geo_point` [airsim_ros_pkgs/GPSYaw](msg/GPSYaw.msg)   
  Listens to home geo coordinates published by `airsim_node`.  

- `/airsim_node/odom_local_ned` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)   
  Listens to odometry published by `airsim_node`

#### Publishers:
- `/vel_cmd_world_frame` [airsim_ros_pkgs/VelCmd](airsim_ros_pkgs/VelCmd)   
  Sends velocity command to `airsim_node`

### Global params
- Dynamic constraints. These can be changed in `dynamic_constraints.launch`:  
    * `/max_vel_horz_abs` [double]   
  Maximum horizontal velocity of the drone (meters/second)

    * `/max_vel_vert_abs` [double]   
  Maximum vertical velocity of the drone (meters/second)
    
    * `/max_yaw_rate_degree` [double]   
  Maximum yaw rate (degrees/second)

## AirSim camera settings 
### Changing camera parameters 
- Frame of reference
The camera positions are defined in a **left-handed coordinate frame** as shown in the image below. +X-axis is along "body front", +Y-axis is along "body right", +Z axis is along "body up" direction.  
![](docs/images/unreal_m210_origin.PNG)

- Stereo   
[This page](https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-) enlists the possible resolutions, and corresponding focal lengths and field of views.   
You can change the default stereo pair pose, image resolution, and _horizontal_ FoV under the "front-left" and "front-right" fields in `Documents\AirSim\Settings.json`. The default parameters are according the `WVGA` settings as [detailed here](https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-).   
More details on AirSim's settings is [available here](https://microsoft.github.io/AirSim/docs/settings/).   
Defaults are (X,Y,Z are in **meters**. ZED's baseline is 12 centimeters, hence we have `-0.06` and `0.06` in Y axis of front_left and front_right):
  * for front-left:
  	```
      "front-left": {
        "CaptureSettings": [
          {
            "ImageType": 0,
            "Width": 672,
            "Height": 376,
            "FOV_Degrees": 87
          }
        ],
        "X": 0.25, "Y": -0.06, "Z": 0.10,
        "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
      },
	```

  * for front-right:
  	```
      "front-right": {
        "CaptureSettings": [
          {
            "ImageType": 0,
            "Width": 672,
            "Height": 376,
            "FOV_Degrees": 87
          }
        ],
        "X": 0.25, "Y": 0.06, "Z": 0.10,
        "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
      }
	```

## Integrating with popular ROS nodes and/or utilities 

### Computing disparity using stereo_image_proc
- `ROS_NAMESPACE=front rosrun stereo_image_proc stereo_image_proc`
- View disparity `rosrun image_view stereo_view stereo:=/front image:=image_rect_color`
- Read stereo_image_proc's [documentation](https://wiki.ros.org/stereo_image_proc)
- Improve disparity/depth: [Choose good stereo params](https://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters)

### Registering RGB images with ground truth depth from AirSim
