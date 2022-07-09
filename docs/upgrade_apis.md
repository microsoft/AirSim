# Upgrading API Client Code
There have been several API changes in AirSim v1.2 that we hope removes inconsistency, adds future extensibility and presents cleaner interface. Many of these changes are however *breaking changes* which means you will need to modify your client code that talks to AirSim.

## Quicker Way
While most changes you need to do in your client code are fairly easy, a quicker way is simply to take a look at the example code such as [Hello Drone](https://github.com/Microsoft/AirSim/tree/main/PythonClient//multirotor/hello_drone.py)or [Hello Car](https://github.com/Microsoft/AirSim/tree/main/PythonClient//car/hello_car.py) to get gist of changes.

## Importing AirSim
Instead of,

```python
from AirSimClient import *
```
use this:

```python
import airsim
```

Above assumes you have installed AirSim module using, 
```
pip install --user airsim
```

If you are running you code from PythonClient folder in repo then you can also do this:

```python
import setup_path 
import airsim
```

Here setup_path.py should exist in your folder and it will set the path of `airsim` package in `PythonClient` repo folder. All examples in PythonClient folder uses this method.

## Using AirSim Classes
As we have everything now in package, you will need to use explicit namespace for AirSim classes like shown below.

Instead of,

```python
client1 = CarClient()
```

use this:

```python
client1 = airsim.CarClient()
```

## AirSim Types

We have moved all types in `airsim` namespace.

Instead of,

```python
image_type = AirSimImageType.DepthVis

d = DrivetrainType.MaxDegreeOfFreedom
```

use this:

```python
image_type = airsim.ImageType.DepthVis

d = airsim.DrivetrainType.MaxDegreeOfFreedom
```

## Getting Images

Nothing new below, it's just combination of above. Note that all APIs that previously took `camera_id`, now takes `camera_name` instead. You can take a look at [available cameras](image_apis.md#avilable_cameras) here.

Instead of,

```python
responses = client.simGetImages([ImageRequest(0, AirSimImageType.DepthVis)])
```

use this:

```python
responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthVis)])
```

## Utility Methods
In earlier version, we provided several utility methods as part of `AirSimClientBase`. These methods are now moved to `airsim` namespace for more pythonic interface.

Instead of,

```python
AirSimClientBase.write_png(my_path, img_rgba) 

AirSimClientBase.wait_key('Press any key')
```

use this:

```python
airsim.write_png(my_path, img_rgba)

airsim.wait_key('Press any key')
```

## Camera Names
AirSim now uses [names](image_apis.md#available_cameras) to reference cameras instead of index numbers. However to retain backward compatibility, these names are aliased with old index numbers as string.

Instead of,

```python
client.simGetCameraInfo(0)
```

use this:

```python
client.simGetCameraInfo("0")

# or

client.simGetCameraInfo("front-center")
```

## Async Methods
For multirotors, AirSim had various methods such as `takeoff` or `moveByVelocityZ` that would take long time to complete. All of such methods are now renamed by adding the suffix *Async* as shown below.

Instead of,

```python
client.takeoff()

client.moveToPosition(-10, 10, -10, 5)
```

use this:

```python
client.takeoffAsync().join()

client.moveToPositionAsync(-10, 10, -10, 5).join()
```

Here `.join()` is a call on Python's `Future` class to wait for the async call to complete. You can also choose to do some other computation instead while the call is in progress.

## Simulation-Only Methods
Now we have clear distinction between methods that are only available in simulation from the ones that may be available on actual vehicle. The simulation only methods are prefixed with `sim` as shown below.

```
getCollisionInfo()      is renamed to       simGetCollisionInfo()
getCameraInfo()         is renamed to       simGetCameraInfo()
setCameraOrientation()  is renamed to       simSetCameraOrientation()
```

## State Information
Previously `CarState` mixed simulation-only information like `kinematics_true`. Moving forward, `CarState` will only contain information that can be obtained in real world.

```python
k = car_state.kinematics_true
```

use this:

```python
k = car_state.kinematics_estimated

# or

k = client.simGetGroundTruthKinematics()
```