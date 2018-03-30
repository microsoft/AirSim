# Image APIs

## Getting a Single Image

Here's a sample code to get a single image. Below returned value is bytes of png format image. To get uncompressed and other format please see next section.

### C++

```cpp
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

int getOneImage() 
{
    using namespace std;
    using namespace msr::airlib;
    
    //for car use CarRpcLibClient
    msr::airlib::MultirotorRpcLibClient client;

    vector<uint8_t> png_image = client.simGetImage(0, VehicleCameraBase::ImageType::Scene);
    //do something with images
}
```

### Python

```cpp
from AirSimClient import *

# for car use CarClient() 
client = MultirotorClient()

png_image = client.simGetImage(0, AirSimImageType.Scene)
# do something with image
```

## Getting Images with More Flexibility

The `simGetImages` API which is slightly more complex to use than `simGetImage` API, for example, you can get left camera view, right camera view and depth image from left camera in a single API call. The `simGetImages` API also allows you to get uncompressed images as well as floating point single channel images (instead of 3 channel (RGB), each 8 bit).

### C++

```cpp
int getStereoAndDepthImages() 
{
    using namespace std;
    using namespace msr::airlib;
    
    typedef VehicleCameraBase::ImageRequest ImageRequest;
    typedef VehicleCameraBase::ImageResponse ImageResponse;
    typedef VehicleCameraBase::ImageType ImageType;

    //for car use
    //msr::airlib::CarRpcLibClient client;
    msr::airlib::MultirotorRpcLibClient client;

    //get right, left and depth images. First two as png, second as float16.
    vector<ImageRequest> request = { 
        //png format
        ImageRequest(0, ImageType::Scene),
        //uncompressed RGBA array bytes
        ImageRequest(1, ImageType::Scene, false, false),       
        //floating point uncompressed image  
        ImageRequest(1, ImageType::DepthPlanner, true) 
    };

    const vector<ImageResponse>& response = client.simGetImages(request);
    //do something with response which contains image data, pose, timestamp etc
}
```

### Python

```python
from AirSimClient import *

# for car use CarClient() 
client = MultirotorClient()

responses = client.simGetImages([
    # png format
    ImageRequest(0, AirSimImageType.Scene), 
    # uncompressed RGBA array bytes
    ImageRequest(1, AirSimImageType.Scene, False, False),
    # floating point uncompressed image
    ImageRequest(1, AirSimImageType.DepthPlanner, True)])
 
 # do something with response which contains image data, pose, timestamp etc
```

#### Using AirSim Images with NumPy

If you plan to use numpy for image manipulation, you should get uncompressed RGBA image and then convert to numpy like this:

```python
responses = client.simGetImages([ImageRequest(0, AirSimImageType.Scene, False, False)])
response = responses[0]

# get numpy array
img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

# reshape array to 4 channel image array H X W X 4
img_rgba = img1d.reshape(response.height, response.width, 4)  

# original image is fliped vertically
img_rgba = np.flipud(img_rgba)

# just for fun add little bit of green in all pixels
img_rgba[:,:,1:2] = 100

# write to png 
Client.write_png(os.path.normpath(filename + '.greener.png'), img_rgba) 
```

## Ready to Run Complete Examples

### C++

For a more complete ready to run sample code please see [sample code in HelloDrone project](../HelloDrone/main.cpp) for multirotors or [HelloCar project](../HelloCar/main.cpp). 

See also [other example code](../Examples/StereoImageGenerator.hpp) that generates specified number of stereo images along with ground truth depth and disparity and saving it to [pfm format](pfm.md).

### Python

For a more complete ready to run sample code please see [sample code in AirSimClient project](../PythonClient/hello_drone.py) for multirotors or [HelloCar sample](../PythonClient/hello_car.py). This code also demonstrates simple activities such as saving images in files or using `numpy` to manipulate images.

## Available Cameras
### Car
The camera ID 0 to 4 corresponds to center front, left front, right front, driver head, center rear respectively.

### Multirotor
The camera ID 0 to 4 corresponds to center front, left front, right front, center downward, center rear respectively.

## "Computer Vision" Mode

You can use AirSim in so-called "Computer Vision" mode. In this mode, physics engine is disabled and there is no flight controller active. This means when you start AirSim, vehicle would just hang in the air. However you can move around using keyboard (use F1 to see help on keys). You can press Record button to continuously generate images. Or you can call APIs to move around and take images.

To active this mode, edit [settings.json](settings.json) that you can find in your `Documents\AirSim` folder (or `~/Documents/AirSim` on Linux) and make sure following values exist at root level:

```json
{
  "SettingsVersion": 1.0,
  "UsageScenario": "ComputerVision"
}
```

When you start AirSim and prompted to choose car or drone, select drone by pressing No button.

[Here's the Python code example](https://github.com/Microsoft/AirSim/blob/master/PythonClient/cv_mode.py) to move camera around and capture images.

If you are only interested in this mode, you might also want to take a look at [UnrealCV project](http://unrealcv.org/).

## How to Set Position and Orientation (Pose)?

### Vehicle Pose
To move around the environment using APIs you can use `simSetPose` API. This API takes position and orientation and sets that on the vehicle. If you don't want to change position (or orientation) then set components of position (or orientation) to floating point nan values.

### Camera Orientation (Gimble)
To change orientation of individial camera, you can use `setCameraOrientation` API. It takes camera ID which is zero-based [index of camera](#available-cameras) and quaternion relative to body in NED frame. For example, to set camera-0 to 15-degree pitch, you can use:
```
client.setCameraOrientation(0, AirSimClientBase.toQuaternion(0.261799, 0, 0)); #radians
```

You can set stabilization for pitch, roll or yaw for any camera [using settings](settings.md#gimble).

Please see [example usage](https://github.com/Microsoft/AirSim/blob/master/PythonClient/cv_mode.py).

## Changing Resolution and Camera Parameters
To change resolution, FOV etc, you can use [settings.json](settings.md). For example, below is the complete content of settings.json that sets parameters for scene capture and uses "Computer Vision" mode described above. If you omit any setting then below default values will be used. For more information see [settings doc](settings.md). If you are using stereo camera, currently the distance between left and right is fixed at 25 cm.

```json
{
  "CaptureSettings": [
    {
      "ImageType": 0,
      "Width": 256,
      "Height": 144,
      "FOV_Degrees": 90,
      "AutoExposureSpeed": 100,
      "MotionBlurAmount": 0
    }
  ],
  "UsageScenario": "ComputerVision"
}
```

### Getting Camera Parameters
The `getCameraInfo(camera_id)` API call returns pose (in world frame, NED coordinates, SI units) and FOV (in degrees) of specified camera. Camera ID is zero-based [index of camera](#available-cameras). Please see [example usage](https://github.com/Microsoft/AirSim/blob/master/PythonClient/cv_mode.py).

## What Does Pixel Values Mean in Different Image Types?
### Available ImageType
```cpp
  Scene = 0, 
  DepthPlanner = 1, 
  DepthPerspective = 2,
  DepthVis = 3, 
  DisparityNormalized = 4,
  Segmentation = 5,
  SurfaceNormals = 6,
  Infrared = 7
```                

### DepthPlanner and DepthPerspective
You normally want to retrieve the depth image as float (i.e. set `pixels_as_float = true`) and specify `ImageType = DepthPlanner` or `ImageType = DepthPerspective` in `ImageRequest`. For `ImageType = DepthPlanner`, you get depth in camera plan, i.e., all points that are in plan parallel to camera have same depth. For `ImageType = DepthPerspective`, you get depth from camera using a projection ray that hits that pixel. Depending on your use case, planner depth or perspective depth may be the ground truth image that you want. For example, you may be able to feed perspective depth to ROS package such as `depth_image_proc` to generate a point cloud. Or planner depth may be more compatible with estimated depth image generated by stereo algorithms such as SGM.

### DepthVis
When you specify `ImageType = DepthVis` in `ImageRequest`, you get an image that helps depth visualization. In this case, each pixel value is interpolated from red to green depending on depth in camera plane in meters. The pixels with pure green means depth of 100m or more while pure red means depth of 0 meters.

### DisparityNormalized
You normally want to retrieve disparity image as float (i.e. set `pixels_as_float = true` and specify `ImageType = DisparityNormalized` in `ImageRequest`) in which case each pixel is `(Xl - Xr)/Xmax`, which is thereby normalized to values between 0 to 1.

### Segmentation
When you specify `ImageType = Segmentation` in `ImageRequest`, you get an image that gives you ground truth segmentation of the scene. At the startup, AirSim assigns value 0 to 255 to each mesh available in environment. This value is than mapped to a specific color in [the pallet](../Unreal/Plugins/AirSim/Content/HUDAssets/seg_color_pallet.png). The RGB values for each object ID can be found in [this file](seg_rgbs.txt).

You can assign a specific value (limited to the range 0-255) to a specific mesh using APIs. For example, below Python code sets the object ID for the mesh called "Ground" to 20 in Blocks environment and hence changes its color in Segmentation view:

```python
success = client.simSetSegmentationObjectID("Ground", 20);
```

The return value is a boolean type that lets you know if the mesh was found.

Notice that typical Unreal environments, like Blocks, usually have many other meshes that comprises of same object, for example, "Ground_2", "Ground_3" and so on. As it is tedious to set object ID for all of these meshes, AirSim also supports regular expressions. For example, the code below sets all meshes which have names starting with "ground" (ignoring case) to 21 with just one line:

```python
success = client.simSetSegmentationObjectID("ground[\w]*", 21, True);
```

The return value is true if at least one mesh was found using regular expression matching.

It is recommended that you request uncompressed image using this API to ensure you get precise RGB values for segmentation image:
```python
responses = client.simGetImages([ImageRequest(0, AirSimImageType.Segmentation, False, False)])
img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
img_rgba = img1d.reshape(response.height, response.width, 4) #reshape array to 4 channel image array H X W X 4
img_rgba = np.flipud(img_rgba) #original image is fliped vertically

#find unique colors
print(np.unique(img_rgba[:,:,0], return_counts=True)) #red
print(np.unique(img_rgba[:,:,1], return_counts=True)) #green
print(np.unique(img_rgba[:,:,2], return_counts=True)) #blue  
```

A complete ready-to-run example can be found in [segmentation.py](https://github.com/Microsoft/AirSim/blob/master/PythonClient/segmentation.py).

#### Unsetting object ID
An object's ID can be set to -1 to make it not show up on the segmentation image.

#### How to Find Mesh Names?
To get desired ground truth segmentation you will need to know the names of the meshes in your Unreal environment. To do this, you will need to open up Unreal Environment in Unreal Editor and then inspect the names of the meshes you are interested in using the World Outliner. For example, below we see the mesh names for he ground in Blocks environment in right panel in the editor:

![record screenshot](images/unreal_editor_blocks.png)

If you don't know how to open Unreal Environment in Unreal Editor then try following the guide for [building from source](build_windows.md).

Once you decide on the meshes you are interested, note down their names and use above API to set their object IDs. There are [few settings](settings.md#segmentation-settings) available to change object ID generation behavior.

#### Changing Colors for Object IDs
At present the color for each object ID is fixed as in [this pallete](../Unreal/Plugins/AirSim/Content/HUDAssets/seg_color_pallet.png). We will be adding ability to change colors for object IDs to desired values shortly. In the meantime you can open the segmentation image in your favorite image editor and get the RGB values you are interested in.

#### Startup Object IDs
At the start, AirSim assigns object ID to each object found in environment of type `UStaticMeshComponent` or `ALandscapeProxy`. It then either uses mesh name or owner name (depending on settings), lower cases it, removes any chars below ASCII 97 to remove numbers and some punctuations, sums int value of all chars and modulo 255 to generate the object ID. In other words, all object with same alphabet chars would get same object ID. This heuristic is simple and effective for many Unreal environments but may not be what you want. In that case, please use above APIs to change object IDs to your desired values. There are [few settings](settings.md#segmentation-settings) available to change this behavior.

#### Getting Object ID for Mesh
The `simGetSegmentationObjectID` API allows you get object ID for given mesh name.

### Infrared
Currently this is just a map from object ID to grey scale 0-255. So any mesh with object ID 42 shows up with color (42, 42, 42). Please see [segmentation section](#segmentation) for more details on how to set object IDs. Typically noise setting can be applied for this image type to get slightly more realistic effect. We are still working on adding other infrared artifacts and any contributions are welcome.

## Collision API
The collision information can be obtained using `getCollisionInfo` API. This call returns a struct that has information not only whether collision occurred but also collision position, surface normal, penetration depth and so on.

## Example Code
A complete example of setting vehicle positions at random locations and orientations and then taking images can be found in [GenerateImageGenerator.hpp](../Examples/StereoImageGenerator.hpp). This example generates specified number of stereo images and ground truth disparity image and saving it to [pfm format](pfm.md).
