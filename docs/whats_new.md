# What's new

Below is highly summerized curated list of important changes. This does not include minor/less important changes or bug fixes or things like documentation update. This list updated every few months. For full list of changes, please review [commit history](https://github.com/Microsoft/AirSim/commits/master).

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
* We now have the [car model](docs/using_car.md).
* No need to build the code. Just download [binaries](https://github.com/Microsoft/AirSim/releases) and you are good to go!
* The [reinforcement learning example](docs/reinforcement_learning.md) with AirSim
* New built-in flight controller called [simple_flight](docs/simple_flight.md) that "just works" without any additional setup. It is also now *default*. 
* AirSim now also generates [depth as well as disparity images](docs/image_apis.md) that is in camera plan. 
* We also have official Linux build now! If you have been using AirSim with PX4, you might want to read the [release notes](docs/release_notes.md).