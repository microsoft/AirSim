# AirSim ROS Tutorials

This is a set of sample AirSim `settings.json`s, roslaunch and rviz files to give a starting point for using AirSim with ROS.     
See [airsim_ros_pkgs](https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_ros_pkgs/README.md) for the ROS API.


## Setup
```shell
$ cd PATH_TO/AirSim/ros
$ catkin build airsim_tutorial_pkgs
```

## Examples

### Single drone with monocular and depth cameras, and lidar
 - Settings.json - [front_stereo_and_center_mono.json](https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_tutorial_pkgs/settings/front_stereo_and_center_mono.json)
 ```shell
 $ source PATH_TO/AirSim/ros/devel/setup.bash
 $ roscd airsim_tutorial_pkgs
 $ cp settings/front_stereo_and_center_mono.json ~/Documents/AirSim/settings.json

 ## Start your unreal package or binary here

 $ roslaunch airsim_ros_pkgs airsim_node.launch;

 # in a new pane / terminal
 $ roslaunch airsim_tutorial_pkgs front_stereo_and_center_mono.launch
 ```
 The above would start rviz with tf's, registered RGBD cloud using [depth_image_proc](https://wiki.ros.org/depth_image_proc) using the [`depth_to_pointcloud` launch file](https://github.com/microsoft/AirSim/master/ros/src/airsim_tutorial_pkgs/launch/front_stereo_and_center_mono/depth_to_pointcloud.launch), and the lidar point cloud. 


### Two drones, with cameras, lidar, IMU each
- Settings.json - [two_drones_camera_lidar_imu.json](https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_tutorial_pkgs/settings/two_drones_camera_lidar_imu.json) 

 ```shell
 $ source PATH_TO/AirSim/ros/devel/setup.bash
 $ roscd airsim_tutorial_pkgs
 $ cp settings/two_drones_camera_lidar_imu.json ~/Documents/AirSim/settings.json

 ## Start your unreal package or binary here

 $ roslaunch airsim_ros_pkgs airsim_node.launch;
 $ roslaunch airsim_ros_pkgs rviz.launch
 ```
You can view the tfs in rviz. And do a `rostopic list` and `rosservice list` to inspect the services avaiable.    

### Twenty-five drones in a square pattern
- Settings.json - [twenty_five_drones.json](https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_tutorial_pkgs/settings/twenty_five_drones.json) 

 ```shell
 $ source PATH_TO/AirSim/ros/devel/setup.bash
 $ roscd airsim_tutorial_pkgs
 $ cp settings/twenty_five_drones.json ~/Documents/AirSim/settings.json

 ## Start your unreal package or binary here

 $ roslaunch airsim_ros_pkgs airsim_node.launch;
 $ roslaunch airsim_ros_pkgs rviz.launch
 ```
You can view the tfs in rviz. And do a `rostopic list` and `rosservice list` to inspect the services avaiable.    
