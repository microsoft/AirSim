# airsim_ros_pkgs

A ROS wrapper over the AirSim C++ client library. 

##  Setup 
- Ubuntu 16.04
  * Install [ROS kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu)

- Ubuntu 18.04
  * Install [ROS melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)

##  Build
- Build AirSim 
```
git clone https://github.com/Microsoft/AirSim.git;
cd AirSim;
./setup.sh;
./build.sh;
```
- Build ROS package

```
cd ros;
catkin_make; # or catkin build
```

## Running
```
source devel/setup.bash;
roslaunch airsim_ros_pkgs airsim_node.launch;
roslaunch airsim_ros_pkgs rviz.launch;
```

# Using AirSim ROS wrapper
The ROS wrapper is composed of two ROS nodes - the first is a wrapper over AirSim's multirotor C++ client library, and the second is a simple PD position controller.    
Let's look at the ROS API for both nodes: 

### AirSim ROS Wrapper Node
#### Publishers:
- `/airsim_node/origin_geo_point` [airsim_ros_pkgs/GPSYaw](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_pkgs/msg/GPSYaw.msg)   
GPS coordinates corresponding to global NED frame. This is set in the airsim's [settings.json](https://microsoft.github.io/AirSim/docs/settings/) file under the `OriginGeopoint` key. 
  
- `/airsim_node/VEHICLE_NAME/global_gps` [sensor_msgs/NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)   
This the current GPS coordinates of the drone in airsim. 

- `/airsim_node/VEHICLE_NAME/odom_local_ned` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)   
Odometry in NED frame wrt take-off point.  
 
- `/airsim_node/VEHICLE_NAME/CAMERA_NAME/IMAGE_TYPE/camera_info` [sensor_msgs/CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

- `/airsim_node/VEHICLE_NAME/CAMERA_NAME/IMAGE_TYPE` [sensor_msgs/Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)   
  RGB or float image depending on image type requested in settings.json.

- `/tf` [tf2_msgs/TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)


#### Subscribers: 
- `/airsim_node/vel_cmd_body_frame` [airsim_ros_pkgs/VelCmd](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_pkgs/msg/VelCmd.msg)    
  Ignore `vehicle_name` field, leave it to blank. We will use `vehicle_name` in future for multiple drones.

- `/airsim_node/vel_cmd_world_frame` [airsim_ros_pkgs/VelCmd](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_pkgs/msg/VelCmd.msg)    
  Ignore `vehicle_name` field, leave it to blank. We will use `vehicle_name` in future for multiple drones.

- `/gimbal_angle_euler_cmd` [airsim_ros_pkgs/GimbalAngleEulerCmd](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_pkgs/msg/GimbalAngleEulerCmd.msg)   
  Gimbal set point in euler angles.    

- `/gimbal_angle_quat_cmd` [airsim_ros_pkgs/GimbalAngleQuatCmd](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_pkgs/msg/GimbalAngleQuatCmd.msg)   
  Gimbal set point in quaternion.    

#### Services:
- `/airsim_node/VEHICLE_NAME/land` [airsim_ros_pkgs/Takeoff](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)

- `/airsim_node/takeoff` [airsim_ros_pkgs/Takeoff](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)

- `/airsim_node/reset` [airsim_ros_pkgs/Reset](https://docs.ros.org/api/std_srvs/html/srv/Empty.html)
 Resets *all* drones

#### Parameters:
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
  * `/pd_position_node/kd_x` [double],   
    `/pd_position_node/kp_y` [double],   
    `/pd_position_node/kp_z` [double],   
    `/pd_position_node/kp_yaw` [double]   
    Proportional gains

  * `/pd_position_node/kd_x` [double],   
    `/pd_position_node/kd_y` [double],   
    `/pd_position_node/kd_z` [double],   
    `/pd_position_node/kd_yaw` [double]   
    Derivative gains

  * `/pd_position_node/reached_thresh_xyz` [double]   
    Threshold euler distance (meters) from current position to setpoint position 

  * `/pd_position_node/reached_yaw_degrees` [double]   
    Threshold yaw distance (degrees) from current position to setpoint position 

- `/pd_position_node/update_control_every_n_sec` [double]   
  Default: 0.01 seconds

#### Services:
- `/airsim_node/VEHICLE_NAME/gps_goal` [Request: [srv/SetGPSPosition](https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_ros_pkgs/srv/SetGPSPosition.srv)]   
  Target gps position + yaw.   
  In **absolute** altitude. 

- `/airsim_node/VEHICLE_NAME/local_position_goal` [Request: [srv/SetLocalPosition](https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_ros_pkgs/srv/SetLocalPosition.srv)   
  Target local position + yaw in global NED frame.   

#### Subscribers:
- `/airsim_node/origin_geo_point` [airsim_ros_pkgs/GPSYaw](https://github.com/microsoft/AirSim/tree/master/ros/src/airsim_ros_pkgs/msg/GPSYaw.msg)   
  Listens to home geo coordinates published by `airsim_node`.  

- `/airsim_node/VEHICLE_NAME/odom_local_ned` [nav_msgs/Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)   
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

### Misc
#### Windows Subsytem for Linux on Windows 10
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
