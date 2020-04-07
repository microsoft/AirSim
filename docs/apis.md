# AirSim APIs

## Introduction
AirSim exposes APIs so you can interact with vehicle in the simulation programmatically. You can use these APIs to retrieve images, get state, control the vehicle and so on.

## Python Quickstart
If you want to use Python to call AirSim APIs, we recommend using Anaconda with Python 3.5 or later versions however some code may also work with Python 2.7 ([help us](CONTRIBUTING.md) improve compatibility!).

First install this package:

```
pip install msgpack-rpc-python
```

You can either get AirSim binaries from [releases](https://github.com/Microsoft/AirSim/releases) or compile from the source ([Windows](build_windows.md), [Linux](build_linux.md)). Once you can run AirSim, choose Car as vehicle and then navigate to `PythonClient\car\` folder and run:

```
python hello_car.py
```

If you are using Visual Studio 2017 then just open AirSim.sln, set PythonClient as startup project and choose `car\hello_car.py` as your startup script.

### Installing AirSim Package
You can also install `airsim` package simply by,

```
pip install airsim
```

You can find source code and samples for this package in `PythonClient` folder in your repo.

**Notes**
1. You may notice a file `setup_path.py` in our example folders. This file has simple code to detect if `airsim` package is available in parent folder and in that case we use that instead of pip installed package so you always use latest code.
2. AirSim is still under heavy development which means you might frequently need to update the package to use new APIs.

## C++ Users
If you want to use C++ APIs and examples, please see [C++ APIs Guide](apis_cpp.md).

## Hello Car
Here's how to use AirSim APIs using Python to control simulated car (see also [C++ example](apis_cpp.md#hello_car)):

```python
# ready to run example: PythonClient/car/hello_car.py
import airsim
import time

# connect to the AirSim simulator
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

while True:
    # get state of the car
    car_state = client.getCarState()
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))

    # set the controls for car
    car_controls.throttle = 1
    car_controls.steering = 1
    client.setCarControls(car_controls)

    # let car drive a bit
    time.sleep(1)

    # get camera images from the car
    responses = client.simGetImages([
        airsim.ImageRequest(0, airsim.ImageType.DepthVis),
        airsim.ImageRequest(1, airsim.ImageType.DepthPlanner, True)])
    print('Retrieved images: %d', len(responses))

    # do something with images
    for response in responses:
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm('py1.pfm', airsim.get_pfm_array(response))
        else:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file('py1.png', response.image_data_uint8)

```

## Hello Drone
Here's how to use AirSim APIs using Python to control simulated quadrotor (see also [C++ example](apis_cpp.md#hello_drone)):

```python
# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
client.moveToPositionAsync(-10, 10, -10, 5).join()

# take images
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),
    airsim.ImageRequest("1", airsim.ImageType.DepthPlanner, True)])
print('Retrieved images: %d', len(responses))

# do something with the images
for response in responses:
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath('/temp/py1.pfm'), airsim.getPfmArray(response))
    else:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath('/temp/py1.png'), response.image_data_uint8)
```

## Common APIs

* `reset`: This resets the vehicle to its original starting state. Note that you must call `enableApiControl` and `armDisarm` again after the call to `reset`.
* `confirmConnection`: Checks state of connection every 1 sec and reports it in Console so user can see the progress for connection.
* `enableApiControl`: For safety reasons, by default API control for autonomous vehicle is not enabled and human operator has full control (usually via RC or joystick in simulator). The client must make this call to request control via API. It is likely that human operator of vehicle might have disallowed API control which would mean that enableApiControl has no effect. This can be checked by `isApiControlEnabled`.
* `isApiControlEnabled`: Returns true if API control is established. If false (which is default) then API calls would be ignored. After a successful call to `enableApiControl`, the `isApiControlEnabled` should return true.
* `ping`: If connection is established then this call will return true otherwise it will be blocked until timeout.
* `simPrintLogMessage`: Prints the specified message in the simulator's window. If message_param is also supplied then its printed next to the message and in that case if this API is called with same message value but different message_param again then previous line is overwritten with new line (instead of API creating new line on display). For example, `simPrintLogMessage("Iteration: ", to_string(i))` keeps updating same line on display when API is called with different values of i. The valid values of severity parameter is 0 to 3 inclusive that corresponds to different colors.
* `simGetObjectPose`, `simSetObjectPose`: Gets and sets the pose of specified object in Unreal environment. Here the object means "actor" in Unreal terminology. They are searched by tag as well as name. Please note that the names shown in UE Editor are *auto-generated* in each run and are not permanent. So if you want to refer to actor by name, you must change its auto-generated name in UE Editor. Alternatively you can add a tag to actor which can be done by clicking on that actor in Unreal Editor and then going to [Tags property](https://answers.unrealengine.com/questions/543807/whats-the-difference-between-tag-and-tag.html), click "+" sign and add some string value. If multiple actors have same tag then the first match is returned. If no matches are found then NaN pose is returned. The returned pose is in NED coordinates in SI units with its origin at Player Start. For `simSetObjectPose`, the specified actor must have [Mobility](https://docs.unrealengine.com/en-us/Engine/Actors/Mobility) set to Movable or otherwise you will get undefined behavior. The `simSetObjectPose` has parameter `teleport` which means object is [moved through other objects](https://www.unrealengine.com/en-US/blog/moving-physical-objects) in its way and it returns true if move was successful

### Image / Computer Vision APIs
AirSim offers comprehensive images APIs to retrieve synchronized images from multiple cameras along with ground truth including depth, disparity, surface normals and vision. You can set the resolution, FOV, motion blur etc parameters in [settings.json](settings.md). There is also API for detecting collision state. See also [complete code](https://github.com/Microsoft/AirSim/tree/master/Examples/DataCollection/StereoImageGenerator.hpp) that generates specified number of stereo images and ground truth depth with normalization to camera plan, computation of disparity image and saving it to [pfm format](pfm.md).

More on [image APIs and Computer Vision mode](image_apis.md).
For vision problems that can benefit from domain randomization, there is also an [object retexturing API](retexturing.md), which can be used in supported scenes.

### Pause and Continue APIs
AirSim allows to pause and continue the simulation through `pause(is_paused)` API. To pause the simulation call `pause(True)` and to continue the simulation call `pause(False)`. You may have scenario, especially while using reinforcement learning, to run the simulation for specified amount of time and then automatically pause. While simulation is paused, you may then do some expensive computation, send a new command and then again run the simulation for specified amount of time. This can be achieved by API `continueForTime(seconds)`. This API runs the simulation for the specified number of seconds and then pauses the simulation. For example usage, please see [pause_continue_car.py](https://github.com/Microsoft/AirSim/tree/master/PythonClient//car/pause_continue_car.py) and [pause_continue_drone.py](https://github.com/Microsoft/AirSim/tree/master/PythonClient//multirotor/pause_continue_drone.py).


### Collision API
The collision information can be obtained using `simGetCollisionInfo` API. This call returns a struct that has information not only whether collision occurred but also collision position, surface normal, penetration depth and so on.

### Time of Day API
AirSim assumes there exist sky sphere of class `EngineSky/BP_Sky_Sphere` in your environment with [ADirectionalLight actor](https://github.com/Microsoft/AirSim/blob/master/Unreal/Plugins/AirSim/Source/SimMode/SimModeBase.cpp#L156). By default, the position of the sun in the scene doesn't move with time. You can use [settings](settings.md#timeofday) to set up latitude, longitude, date and time which AirSim uses to compute the position of sun in the scene.

You can also use following API call to set the sun position according to given date time:

```
simSetTimeOfDay(self, is_enabled, start_datetime = "", is_start_datetime_dst = False, celestial_clock_speed = 1, update_interval_secs = 60, move_sun = True)
```

The `is_enabled` parameter must be `True` to enable time of day effect. If it is `False` then sun position is reset to its original in the environment.

Other parameters are same as in [settings](settings.md#timeofday).

### Weather APIs
By default all weather effects are disabled. To enable weather effect, first call:

```
simEnableWeather(True)
```

Various weather effects can be enabled by using `simSetWeatherParameter` method which takes `WeatherParameter`, for example,

```
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.25);
```
The second parameter value is from 0 to 1. The first parameter provides following options:

```
class WeatherParameter:
    Rain = 0
    Roadwetness = 1
    Snow = 2
    RoadSnow = 3
    MapleLeaf = 4
    RoadLeaf = 5
    Dust = 6
    Fog = 7
```

Please note that `Roadwetness`, `RoadSnow` and `RoadLeaf` effects requires adding [materials](https://github.com/Microsoft/AirSim/tree/master/Unreal/Plugins/AirSim/Content/Weather/WeatherFX) to your scene.

Please see [example code](https://github.com/Microsoft/AirSim/blob/master/PythonClient/computer_vision/weather.py) for more details.

### Lidar APIs
AirSim offers API to retrieve point cloud data from Lidar sensors on vehicles. You can set the number of channels, points per second, horizontal and vertical FOV, etc parameters in [settings.json](settings.md).

More on [lidar APIs and settings](lidar.md) and [sensor settings](sensors.md)

### Multiple Vehicles
AirSim supports multiple vehicles and control them through APIs. Please [Multiple Vehicles](multi_vehicle.md) doc.

### Coordinate System
All AirSim API uses NED coordinate system, i.e., +X is North, +Y is East and +Z is Down. All units are in SI system. Please note that this is different from coordinate system used internally by Unreal Engine. In Unreal Engine, +Z is up instead of down and length unit is in centimeters instead of meters. AirSim APIs takes care of the appropriate conversions. The starting point of the vehicle is always coordinates (0, 0, 0) in NED system. Thus when converting from Unreal coordinates to NED, we first subtract the starting offset and then scale by 100 for cm to m conversion. The vehicle is spawned in Unreal environment where the Player Start component is placed. There is a setting called `OriginGeopoint` in [settings.json](settings.md) which assigns geographic longitude, longitude and altitude to the Player Start component.

## Vehicle Specific APIs
### APIs for Car
Car has followings APIs available:

* `setCarControls`: This allows you to set throttle, steering, handbrake and auto or manual gear.
* `getCarState`: This retrieves the state information including speed, current gear and 6 kinematics quantities: position, orientation, linear and angular velocity, linear and angular acceleration. All quantities are in NED coordinate system, SI units in world frame except for angular velocity and accelerations which are in body frame.
* [Image APIs](image_apis.md).

### APIs for Multirotor
Multirotor can be controlled by specifying angles, velocity vector, destination position or some combination of these. There are corresponding `move*` APIs for this purpose. When doing position control, we need to use some path following algorithm. By default AirSim uses carrot following algorithm. This is often referred to as "high level control" because you just need to specify high level goal and the firmware takes care of the rest. Currently lowest level control available in AirSim is `moveByAngleThrottleAsync` API.

#### getMultirotorState
This API returns the state of the vehicle in one call. The state includes, collision, estimated kinematics (i.e. kinematics computed by fusing sensors), and timestamp (nano seconds since epoch). The kinematics here means 6 quantities: position, orientation, linear and angular velocity, linear and angular acceleration. Please note that simple_slight currently doesn't support state estimator which means estimated and ground truth kinematics values would be same for simple_flight. Estimated kinematics are however available for PX4 except for angular acceleration. All quantities are in NED coordinate system, SI units in world frame except for angular velocity and accelerations which are in body frame.

#### Async methods, duration and max_wait_seconds
Many API methods has parameters named `duration` or `max_wait_seconds` and they have *Async* as suffix, for example, `takeoffAsync`. These methods will return immediately after starting the task in AirSim so that your client code can do something else while that task is being executed. If you want to wait for this task to complete then you can call `waitOnLastTask` like this:

```cpp
//C++
client.takeoffAsync()->waitOnLastTask();
```

```cpp
# Python
client.takeoffAsync().join()
```

If you start another command then it automatically cancels the previous task and starts new command. This allows to use pattern where your coded continuously does the sensing, computes a new trajectory to follow and issues that path to vehicle in AirSim. Each newly issued trajectory cancels the previous trajectory allowing your code to continuously do the update as new sensor data arrives.

All *Async* method returns `concurrent.futures.Future` in Python (`std::future` in C++). Please note that these future classes currently do not allow to check status or cancel the task; they only allow to wait for task to complete. AirSim does provide API `cancelLastTask`, however.

#### drivetrain
There are two modes you can fly vehicle: `drivetrain` parameter is set to `airsim.DrivetrainType.ForwardOnly` or `airsim.DrivetrainType.MaxDegreeOfFreedom`. When you specify ForwardOnly, you are saying that vehicle's front should always point in the direction of travel. So if you want drone to take left turn then it would first rotate so front points to left. This mode is useful when you have only front camera and you are operating vehicle using FPV view. This is more or less like travelling in car where you always have front view. The MaxDegreeOfFreedom means you don't care where the front points to. So when you take left turn, you just start going left like crab. Quadrotors can go in any direction regardless of where front points to. The MaxDegreeOfFreedom enables this mode.

#### yaw_mode
`yaw_mode` is a struct `YawMode` with two fields, `yaw_or_rate` and `is_rate`. If `is_rate` field is True then `yaw_or_rate` field is interpreted as angular velocity in degrees/sec which means you want vehicle to rotate continuously around its axis at that angular velocity while moving. If `is_rate` is False then `yaw_or_rate` is interpreted as angle in degrees which means you want vehicle to rotate to specific angle (i.e. yaw) and keep that angle while moving.

You can probably see that when `yaw_mode.is_rate == true`, the `drivetrain` parameter shouldn't be set to `ForwardOnly` because you are contradicting by saying that keep front pointing ahead but also rotate continuously. However if you have `yaw_mode.is_rate = false` in `ForwardOnly` mode then you can do some funky stuff. For example, you can have drone do circles and have yaw_or_rate set to 90 so camera is always pointed to center ("super cool selfie mode"). In `MaxDegreeofFreedom` also you can get some funky stuff by setting `yaw_mode.is_rate = true` and say `yaw_mode.yaw_or_rate = 20`. This will cause drone to go in its path while rotating which may allow to do 360 scanning.

In most cases, you just don't want yaw to change which you can do by setting yaw rate of 0. The shorthand for this is `airsim.YawMode.Zero()` (or in C++: `YawMode::Zero()`).

#### lookahead and adaptive_lookahead
When you ask vehicle to follow a path, AirSim uses "carrot following" algorithm. This algorithm operates by looking ahead on path and adjusting its velocity vector. The parameters for this algorithm is specified by `lookahead` and `adaptive_lookahead`. For most of the time you want algorithm to auto-decide the values by simply setting `lookahead = -1` and `adaptive_lookahead = 0`.

## Using APIs on Real Vehicles
We want to be able to run *same code* that runs in simulation as on real vehicle. This allows you to test your code in simulator and deploy to real vehicle.

Generally speaking, APIs therefore shouldn't allow you to do something that cannot be done on real vehicle (for example, getting the ground truth). But, of course, simulator has much more information and it would be useful in applications that may not care about running things on real vehicle. For this reason, we clearly delineate between sim-only APIs by attaching `sim` prefix, for example, `simGetGroundTruthKinematics`. This way you can avoid using these simulation-only APIs if you care about running your code on real vehicles.

The AirLib is self-contained library that you can put on an offboard computing module such as the Gigabyte barebone Mini PC. This module then can talk to the flight controllers such as PX4 using exact same code and flight controller protocol. The code you write for testing in the simulator remains unchanged. See [AirLib on custom drones](custom_drone.md).

## Adding New APIs to AirSim
Adding new APIs requires modifying the source code. Much of the changes are mechanical and required for various levels of abstractions that AirSim supports. [This commit](https://github.com/Microsoft/AirSim/commit/f0e83c29e7685e1021185e3c95bfdaffb6cb85dc) demonstrates how to add a simple API `simPrintLogMessage` that prints message in simulator window.

## Some Internals
The APIs use [msgpack-rpc protocol](https://github.com/msgpack-rpc/msgpack-rpc) over TCP/IP through [rpclib](http://rpclib.net/) developed by [TamÃ¡s Szelei](https://github.com/sztomi) which allows you to use variety of programming languages including C++, C#, Python, Java etc. When AirSim starts, it opens port 41451 (this can be changed via [settings](settings.md)) and listens for incoming request. The Python or C++ client code connects to this port and sends RPC calls using [msgpack serialization format](https://msgpack.org).

## References and Examples

* [C++ API Examples](apis_cpp.md)
* [Car Examples](https://github.com/Microsoft/AirSim/tree/master/PythonClient//car)
* [Multirotor Examples](https://github.com/Microsoft/AirSim/tree/master/PythonClient//multirotor)
* [Computer Vision Examples](https://github.com/Microsoft/AirSim/tree/master/PythonClient//computer_vision)
* [Move on Path](https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo) demo showing video of fast multirotor flight through Modular Neighborhood environment
* [Building a Hexacopter](https://github.com/Microsoft/AirSim/wiki/hexacopter)
* [Building Point Clouds](https://github.com/Microsoft/AirSim/wiki/Point-Clouds)


## FAQ

#### Unreal is slowed down dramatically when I run API
If you see Unreal getting slowed down dramatically when Unreal Engine window loses focus then go to 'Edit->Editor Preferences' in Unreal Editor, in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

#### Do I need anything else on Windows?
You should install VS2017 with VC++, Windows SDK 8.1 and Python. To use Python APIs you will need Python 3.5 or later (install it using Anaconda).

#### Which version of Python should I use?
We recommend [Anaconda](https://www.anaconda.com/download/) to get Python tools and libraries. Our code is tested with Python 3.5.3 :: Anaconda 4.4.0. This is important because older version have been known to have [problems](https://stackoverflow.com/a/45934992/207661).

#### I get error on `import cv2`
You can install OpenCV using:
```
conda install opencv
pip install opencv-python
```

#### TypeError: unsupported operand type(s) for *: 'AsyncIOLoop' and 'float'

This error happens if you install Jupyter, which somehow breaks the msgpackrpc library.  Create a new python environment
which the minimal required packages.