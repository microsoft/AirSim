# What's new

Below is summarized list of important changes. This does not include minor/less important changes or bug fixes or documentation update. This list updated every few months. For complete detailed changes, please review [commit history](https://github.com/Microsoft/AirSim/commits/main).


### Jan 2022
* Latest release `v1.7.0` for [Windows](https://github.com/microsoft/AirSim/releases/tag/v1.7.0-windows) and [Linux](https://github.com/microsoft/AirSim/releases/tag/v1.7.0-linux)

### Dec 2021
* [Cinematographic Camera](https://github.com/microsoft/AirSim/pull/3949)
* [ROS2 wrapper](https://github.com/microsoft/AirSim/pull/3976)
* [API to list all assets](https://github.com/microsoft/AirSim/pull/3940)
* [movetoGPS API](https://github.com/microsoft/AirSim/pull/3746)

### Nov 2021
* [Optical flow camera](https://github.com/microsoft/AirSim/pull/3938)
* [simSetKinematics API](https://github.com/microsoft/AirSim/pull/4066)
* [Dynamically set object textures from existing UE material or texture PNG](https://github.com/microsoft/AirSim/pull/3992)
* [Ability to spawn/destroy lights and control light parameters](https://github.com/microsoft/AirSim/pull/3991)

### Sep 2021
* [Support for multiple drones in Unity](https://github.com/microsoft/AirSim/pull/3128)
### Aug 2021
* [Control manual camera speed through the keyboard](https://github.com/microsoft/AirSim/pulls?page=6&q=is%3Apr+is%3Aclosed+sort%3Aupdated-desc#:~:text=1-,Control%20manual%20camera%20speed%20through%20the%20keyboard,-%233221%20by%20saihv) 
* Latest release `v1.6.0` for [Windows](https://github.com/microsoft/AirSim/releases/tag/v1.6.0-windows) and [Linux](https://github.com/microsoft/AirSim/releases/tag/v1.6.0-linux)
* [Fix: DepthPlanar capture](https://github.com/microsoft/AirSim/pull/3907)
* [Fix: compression bug in segmentation palette](https://github.com/microsoft/AirSim/pull/3937)

### Jul 2021
* [Fixed external cameras](https://github.com/microsoft/AirSim/pull/3320)
* [Fix: ROS topic names](https://github.com/microsoft/AirSim/pull/3880)
* [Fix: Weather API crash](https://github.com/microsoft/AirSim/pull/3009)

### Jun 2021
* [Object detection API](https://github.com/microsoft/AirSim/pull/3472)
* [GazeboDrone project added to connect a gazebo drone to the AirSim drone](https://github.com/microsoft/AirSim/pull/3754)
* [Control manual camera speed through the keyboard](https://github.com/microsoft/AirSim/pull/3221)
* [Octo X config](https://github.com/microsoft/AirSim/pull/3653)
* [API for list of vehicle names](https://github.com/microsoft/AirSim/pull/2936)
* [Fix: issue where no new scene is rendered after simContinueForTime](https://github.com/microsoft/AirSim/pull/3305)
* [Fix:Check for settings.json in current directory as well](https://github.com/microsoft/AirSim/pull/3436)

### May 2021
* [Make falling leaves visible in depth and segmentation](https://github.com/microsoft/AirSim/pull/3699)
* [Fix: Unity Car API](https://github.com/microsoft/AirSim/pull/2937)
* Latest release `v1.5.0` for [Windows](https://github.com/microsoft/AirSim/releases/tag/v1.5.0-windows) and [Linux](https://github.com/microsoft/AirSim/releases/tag/v1.5.0-linux)
* [fix px4 connection for wsl 2.](https://github.com/microsoft/AirSim/pull/3603)

### Apr 2021

* [External physics engine](https://github.com/microsoft/AirSim/pull/3626)
* [ArduPilot Sensor Updates](https://github.com/microsoft/AirSim/pull/3364)
* [Add new build configuration "--RelWithDebInfo" which makes it easier to debug](https://github.com/microsoft/AirSim/pull/3596)
* [Add ApiServerPort to available AirSim settings](https://github.com/microsoft/AirSim/pull/3196)
* [ROS: Use the same settings as AirSim](https://github.com/microsoft/AirSim/pull/3536)

### Mar 2021

* [Add moveByVelocityZBodyFrame](https://github.com/microsoft/AirSim/pull/3475)
* [Spawn vehicles via RPC](https://github.com/microsoft/AirSim/pull/2390)
* [Unity weather parameters, weather HUD, and a visual effect for snow](https://github.com/microsoft/AirSim/pull/2909)
* [Rotor output API](https://github.com/microsoft/AirSim/pull/3242)
* [Extend Recording to multiple vehicles](https://github.com/microsoft/AirSim/pull/2861)
* [Combine Lidar Segmentation API into getLidarData](https://github.com/microsoft/AirSim/pull/2810)

### Feb 2021

* [Add Ubuntu 20.04 to Actions CI](https://github.com/microsoft/AirSim/pull/3383)
* [add tcp server support to MavLinkTest](https://github.com/microsoft/AirSim/pull/3386)

### Jan 2021

* [Added continueForFrames](https://github.com/microsoft/AirSim/pull/3102)
* Latest release `v1.4.0` for [Windows](https://github.com/microsoft/AirSim/releases/tag/v1.4.0-windows) and [Linux](https://github.com/microsoft/AirSim/releases/tag/v1.4.0-linux)

### Dec 2020

* [Add Actions script to build and deploy to gh-pages](https://github.com/microsoft/AirSim/pull/3224)
* [Gym environments and stable-baselines integration for RL](https://github.com/microsoft/AirSim/pull/3215)
* [Programmable camera distortion](https://github.com/microsoft/AirSim/pull/3039)
* [Voxel grid construction](https://github.com/microsoft/AirSim/pull/3209)
* [Event camera simulation](https://github.com/microsoft/AirSim/pull/3202)
* [Add Github Actions CI Checks](https://github.com/microsoft/AirSim/pull/3180)
* [Added moveByVelocityBodyFrame](https://github.com/microsoft/AirSim/pull/3169)

### Nov 2020

* [fix auto-detect of pixhawk 4 hardware](https://github.com/microsoft/AirSim/pull/3156)

### Oct 2020

* [\[Travis\] Add Ubuntu 20.04, OSX XCode 11.5 jobs](https://github.com/microsoft/AirSim/pull/2953)

### Sep 2020

* [Add Vehicle option for Subwindow settings](https://github.com/microsoft/AirSim/pull/2841)
* [Disable cameras after fetching images, projection matrix](https://github.com/microsoft/AirSim/pull/2881)
* [Add Wind simulation](https://github.com/microsoft/AirSim/pull/2867)
* [New `simRunConsoleCommand` API](https://github.com/microsoft/AirSim/pull/2996)
* [UE4: Fixes and improvements to World APIs](https://github.com/microsoft/AirSim/pull/2898)
* [UE4: Fix random crash with Plotting APIs](https://github.com/microsoft/AirSim/pull/2963)
* [Add backwards-compatibility layer for `simSetCameraPose`](https://github.com/microsoft/AirSim/pull/2932)
* [Disable LogMessages if set to false](https://github.com/microsoft/AirSim/pull/2946)
* [ROS: Removed double inclusion of `static_transforms.launch`](https://github.com/microsoft/AirSim/pull/2939)
* [Add retry loop when connecting PX4 SITL control channel](https://github.com/microsoft/AirSim/pull/2986)
* [Allow for enabling physics when spawning a new object](https://github.com/microsoft/AirSim/pull/2902)

### July 2020

* [Add Python APIs for new Object functions](https://github.com/microsoft/AirSim/pull/2897)
* [UE4: Fix Broken List Level Option](https://github.com/microsoft/AirSim/pull/2877)
* [Linux build improvements](https://github.com/microsoft/AirSim/pull/2522)
* [Allow passing the settings.json file location via `--settings` argument](https://github.com/microsoft/AirSim/pull/2668)
* [Distance Sensor Upgrades and fixes](https://github.com/microsoft/AirSim/pull/2807)
* [Update to min CMake version required for VS 2019](https://github.com/microsoft/AirSim/pull/2766)
* [Fix: Non-linear bias corrupts SurfaceNormals, Segmentation image](https://github.com/microsoft/AirSim/pull/2845)
* [Fix: `simGetSegmentationObjectID` will always return -1](https://github.com/microsoft/AirSim/pull/2855)
* [Initial implementation of simLoadLevel, simGet/SetObjectScale, simSpawn|DestroyObject APIs](https://github.com/microsoft/AirSim/pull/2651)
* [Upgrade `setCameraOrientation` API to `setCameraPose`](https://github.com/microsoft/AirSim/pull/2710)
* [ROS: All sensors and car support](https://github.com/microsoft/AirSim/pull/2743)
* [Get rid of potential div-0 errors so we can set dt = 0 for pausing](https://github.com/microsoft/AirSim/pull/2705)
* [ROS: Add mavros_msgs to build dependencies](https://github.com/microsoft/AirSim/pull/2642)
* [Move Wiki pages to docs](https://github.com/microsoft/AirSim/pull/2803)
* [Add Recording APIs](https://github.com/microsoft/AirSim/pull/2834)
* [Update Dockerfiles and documentation to Ubuntu 18.04](https://github.com/microsoft/AirSim/pull/2865)
* [Azure development environment and documentation](https://github.com/microsoft/AirSim/pull/2816)
* [ROS: Add airsim_node to install list](https://github.com/microsoft/AirSim/pull/2706)

### May 2020

* [Fix more issues with PX4 master](https://github.com/microsoft/AirSim/pull/2649)
* [Reduce warnings level in Unity build](https://github.com/microsoft/AirSim/pull/2672)
* [Support for Unreal Engine 4.25](https://github.com/microsoft/AirSim/pull/2669)
* [Unity crash fix, upgrade to 2019.3.12, Linux build improvements](https://github.com/microsoft/AirSim/pull/2328)

### April 2020

* [Fix issues with PX4 latest master branch](https://github.com/microsoft/AirSim/pull/2634)
* [Fix Lidar DrawDebugPoints causing crash](https://github.com/microsoft/AirSim/pull/2614)
* [Add docstrings for Python API](https://github.com/microsoft/AirSim/pull/2565)
* [Add missing noise, weather texture materials](https://github.com/microsoft/AirSim/pull/2625)
* [Update AirSim.uplugin version to 1.3.1](https://github.com/microsoft/AirSim/pull/2584)
* [Camera Roll angle control using Q,E keys in CV mode, manual camera](https://github.com/microsoft/AirSim/pull/2610)
* [Remove broken GCC build](https://github.com/microsoft/AirSim/pull/2572)
* New API - [`simSetTraceLine()`](https://github.com/microsoft/AirSim/pull/2506)
* [ROS package compilation fixes and updates](https://github.com/microsoft/AirSim/pull/2571)
* Latest release `v1.3.1` for [Windows](https://github.com/microsoft/AirSim/releases/tag/v1.3.1-windows) and [Linux](https://github.com/microsoft/AirSim/releases/tag/v1.3.1-linux)
* APIs added and fixed - [`simSetCameraFov`](https://github.com/microsoft/AirSim/pull/2534), [`rotateToYaw`](https://github.com/microsoft/AirSim/pull/2516)
* [airsim](https://pypi.org/project/airsim/) Python package update to `1.2.8`
* [NoDisplay ViewMode render state fix](https://github.com/microsoft/AirSim/pull/2518)

### March 2020

* Latest release `v1.3.0` for [Windows](https://github.com/microsoft/AirSim/releases/tag/v1.3.0-Windows) and [Linux](https://github.com/microsoft/AirSim/releases/tag/v1.3.0-linux)
* Upgraded to Unreal Engine 4.24, Visual Studio 2019, Clang 8, C++ 17 standard
* Mac OSX Catalina support
* Updated [airsim](https://pypi.org/project/airsim/) Python package, with lots of new APIs
* [Removed legacy API wrappers](https://github.com/microsoft/AirSim/pull/2494)
* [Support for latest PX4 stable release](px4_setup.md)
* Support for [ArduPilot](https://ardupilot.org/ardupilot/) - [Copter, Rover vehicles](https://ardupilot.org/dev/docs/sitl-with-airsim.html)
* [Updated Unity support](Unity.md)
* [Removed simChar* APIs](https://github.com/microsoft/AirSim/pull/2493)
* [Plotting APIs for Debugging](https://github.com/microsoft/AirSim/pull/2304)
* [Low-level Multirotor APIs](https://github.com/microsoft/AirSim/pull/2297)
* [Updated Eigen version to 3.3.7](https://github.com/microsoft/AirSim/pull/2325)
* [Distance Sensor API fix](https://github.com/microsoft/AirSim/pull/2403)
* Add [`simSwapTextures`](retexturing.md) API
* Fix [`simContinueForTime`](https://github.com/microsoft/AirSim/pull/2299), [`simPause`](https://github.com/microsoft/AirSim/pull/2292) APIs
* [Lidar Sensor Trace Casting fix](https://github.com/microsoft/AirSim/pull/2143)
* [Fix rare `reset()` bug which causes Unreal crash](https://github.com/microsoft/AirSim/pull/2146)
* [Lidar sensor improvements, add `simGetLidarSegmentation` API](https://github.com/microsoft/AirSim/pull/2011)
* [Add RpcLibPort in settings](https://github.com/microsoft/AirSim/pull/2185)
* [Recording thread deadlock fix](https://github.com/microsoft/AirSim/pull/1695)
* [Prevent environment crash when Sun is not present](https://github.com/microsoft/AirSim/pull/2147)
* [Africa Tracking feautre, add `simListSceneObjects()` API, fix camera projection matrix](https://github.com/microsoft/AirSim/pull/1959)
* ROS wrapper for multirotors is available. See [airsim_ros_pkgs](airsim_ros_pkgs.md) for the ROS API, and [airsim_tutorial_pkgs](airsim_tutorial_pkgs.md) for tutorials.
* [Added sensor APIs for Barometer, IMU, GPS, Magnetometer, Distance Sensor](sensors.md)
* Added support for [docker in ubuntu](docker_ubuntu.md)

### November, 2018
* Added Weather Effects and [APIs](apis.md#weather-apis)
* Added [Time of Day API](apis.md#time-of-day-api)
* An experimental integration of [AirSim on Unity](https://github.com/Microsoft/AirSim/tree/main/Unity) is now available. Learn more in [Unity blog post](https://blogs.unity3d.com/2018/11/14/airsim-on-unity-experiment-with-autonomous-vehicle-simulation). 
* [New environments](https://github.com/Microsoft/AirSim/releases/tag/v1.2.1): Forest, Plains (windmill farm), TalkingHeads (human head simulation), TrapCam (animal detection via camera)
* Highly efficient [NoDisplay view mode](settings.md#viewmode) to turn off main screen rendering so you can capture images at high rate
* [Enable/disable sensors](https://github.com/Microsoft/AirSim/pull/1479) via settings
* [Lidar Sensor](lidar.md)
* [Support for Flysky FS-SM100 RC](https://github.com/Microsoft/AirSim/commit/474214364676b6631c01b3ed79d00c83ba5bccf5) USB adapter
* Case Study: [Formula Student Technion Driverless](https://github.com/Microsoft/AirSim/wiki/technion)
* [Multi-Vehicle Capability](multi_vehicle.md)
* [Custom speed units](https://github.com/Microsoft/AirSim/pull/1181)
* [ROS publisher](https://github.com/Microsoft/AirSim/pull/1135)
* [simSetObjectPose API](https://github.com/Microsoft/AirSim/pull/1161)
* [Character Control APIs](https://github.com/Microsoft/AirSim/blob/main/PythonClient/airsim/client.py#L137) (works on TalkingHeads binaries in release)
* [Arducopter Solo Support](https://github.com/Microsoft/AirSim/pull/1387)
* [Linux install without sudo access](https://github.com/Microsoft/AirSim/pull/1434)
* [Kinect like ROS publisher](https://github.com/Microsoft/AirSim/pull/1298)


### June, 2018
* Development workflow doc
* Better Python 2 compatibility
* OSX setup fixes
* Almost complete rewrite of our APIs with new threading model, merging old APIs and creating few newer ones

### April, 2018
* Upgraded to Unreal Engine 4.18 and Visual Studio 2017
* API framework refactoring to support world-level APIs
* Latest PX4 firmware now supported
* CarState with more information
* ThrustMaster wheel support
* pause and continueForTime APIs for drone as well as car
* Allow drone simulation run at higher clock rate without any degradation
* Forward-only mode fully functional for drone (do orbits while looking at center)
* Better PID tuning to reduce wobble for drones
* Ability to set arbitrary vehicle blueprint for drone as well as car
* gimbal stabilization via settings
* Ability to segment skinned and skeletal meshes by their name
* moveByAngleThrottle API
* Car physics tuning for better maneuverability
* Configure additional cameras via settings
* Time of day with geographically computed sun position
* Better car steering via keyboard
* Added MeshNamingMethod in segmentation setting 
* gimbal API
* getCameraParameters API
* Ability turn off main rendering to save GPU resources
* Projection mode for capture settings
* getRCData, setRCData APIs
* Ability to turn off segmentation using negative IDs
* OSX build improvements
* Segmentation working for very large environments with initial IDs
* Better and extensible hash calculation for segmentation IDs
* Extensible PID controller for custom integration methods
* Sensor architecture now enables renderer specific features like ray casting
* Laser altimeter sensor


### Jan 2018
* Config system rewrite, enable flexible config we are targeting in future
* Multi-Vehicle support Phase 1, core infrastructure changes
* MacOS support
* Infrared view
* 5 types of noise and interference for cameras
* WYSIWIG capture settings for cameras, preview recording settings in main view
* Azure support Phase 1, enable configurability of instances for headless mode
* Full kinematics APIs, ability to get pose, linear and angular velocities + accelerations via APIs
* Record multiple images from multiple cameras
* New segmentation APIs, ability to set configure object IDs, search via regex
* New object pose APIs, ability to get pose of objects (like animals) in environment
* Camera infrastructure enhancements, ability to add new image types like IR with just few lines
* Clock speed APIs for drone as well as car, simulation can be run with speed factor of 0 < x < infinity
* Support for Logitech G920 wheel
* Physics tuning of the car, Car doesn’t roll over, responds to steering with better curve, releasing gas paddle behavior more realistic
* Debugging APIs
* Stress tested to 24+ hours of continuous runs
* Support for Landscape and sky segmentation
* Manual navigation with accelerated controls in CV mode, user can explore environment much more easily
* Collison APIs
* Recording enhancements, log several new data points including ground truth, multiple images, controls state
* Planner and Perspective Depth views
* Disparity view
* New Image APIs supports float, png or numpy formats
* 6 config settings for image capture, ability to set auto-exposure, motion blur, gamma etc
* Full multi-camera support through out including sub-windows, recording, APIs etc
* Command line script to build all environments in one shot
* Remove submodules, use rpclib as download

### Nov 2017
* We now have the [car model](using_car.md).
* No need to build the code. Just download [binaries](https://github.com/Microsoft/AirSim/releases) and you are good to go!
* The [reinforcement learning example](reinforcement_learning.md) with AirSim
* New built-in flight controller called [simple_flight](simple_flight.md) that "just works" without any additional setup. It is also now *default*. 
* AirSim now also generates [depth as well as disparity images](image_apis.md) that are in camera plane. 
* We also have official Linux build now!

## Sep 2017
- We have added [car model](using_car.md)!

## Aug 2017
- [simple_flight](simple_flight.md) is now default flight controller for drones. If you want to use PX4, you will need to modify settings.json as per [PX4 setup doc](px4_setup.md).
- Linux build is official and currently uses Unreal 4.17 due to various bug fixes required
- ImageType enum has breaking changes with several new additions and clarifying existing ones
- SubWindows are now configurable from settings.json
- PythonClient is now complete and has parity with C++ APIs. Some of these would have breaking changes.

## Feb 2017
- First release!
