# Image APIs

## Getting a Single Image

Here's a sample code to get a single image. Below returned value is bytes of png format image. To get uncompressed and other format please see next section.

### C++

```
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

```
from AirSimClient import *

# for car use CarClient() 
client = MultirotorClient()

png_image = client.simGetImage(0, AirSimImageType.Scene)
# do something with image
```

## Getting Images with More Flexibility

The `simGetImages` API which is slightly more complex to use than `simGetImage` API, for example, you can get left camera view, right camera view and depth image from left camera in a single API call. The `simGetImages` API also allows you to get uncompressed images as well as floating point single channel images (instead of 3 channel (RGB), each 8 bit).

### C++

```
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

```
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

```
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

## "Computer Vision" Mode

You can use AirSim in so-called "Computer Vision" mode. In this mode, physics engine is disabled and there is no flight controller active. This means when you start AirSim, vehicle would just hang in the air. However you can move around using keyboard (use F1 to see help on keys). You can press Record button to continuously generate images. Or you can call APIs to move around and take images.

To active this mode, edit [settings.json](settings.json) that you can find in your `Documents\AirSim` folder (or `~/Documents/AirSim` on Linux) and make sure following values exist at root level:

```
{
  "SettingsVersion": 1.0,
  "UsageScenario": "ComputerVision"
}
```

[Here's the Python code example](https://github.com/Microsoft/AirSim/blob/master/PythonClient/cv_mode.py) to move camera around and capture images.

If you are only interested in this mode, you might also want to take a look at [UnrealCV project](http://unrealcv.org/).

## How to Set Position and Orientation (Pose)?

To move around the environment using APIs you can use `simSetPose` API. This API takes position and orientation and sets that on the vehicle. If you don't want to change position (or orientation) then set components of position (or orientation) to floating point nan values.

## Changing Resolution and Camera Parameters
To change resolution, FOV etc, you can use [settings.json](settings.md). For example, below is the complete content of settings.json that sets parameters for scene capture and uses "Computer Vision" mode described above. If you omit any setting then below default values will be used. For more information see [settings doc](settings.md). If you are using stereo camera, currently the distance between left and right is fixed at 25 cm.

```
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

## What Does Pixel Values Mean in Different Image Types?
### Available ImageType
```
  Scene = 0, 
  DepthPlanner = 1, 
  DepthPerspective = 2,
  DepthVis = 3, 
  DisparityNormalized = 4,
  Segmentation = 5,
  SurfaceNormals = 6
```                

### DepthPlanner and DepthPerspective
You normally want retrieve depth image as float (i.e. set `pixels_as_float = true`) and specify `ImageType = DepthPlanner` or `ImageType = DepthPerspective` in `ImageRequest`. For `ImageType = DepthPlanner`, you get depth in camera plan, i.e., all points that are in plan parallel to camera have same depth. For `ImageType = DepthPerspective`, you get depth from camera using a project ray that hits that pixel. Depending on your use case, planner depth or perspective depth may be the ground truth image that you want. For example, you may be able to feed perspective depth to ROS package such as `depth_image_proc` to generate point cloud. Or planner depth may be more compatible with estimated depth image generated by stereo algorithms such as SGM.

### DepthVis
When you specify `ImageType = DepthVis` in `ImageRequest`, you get image that helps depth visualization. In this case, each pixel value is interpolated from red to green depending on depth in camera plane in meters. The pixels with pure green means depth of 100m or more while pure red means depth of 0 meters.

### DisparityNormalized
You normally want retrieve disparity image as float (i.e. set `pixels_as_float = true` and specify `ImageType = DisparityNormalized` in `ImageRequest`) in which case each pixel is `(Xl - Xr)/Xmax`, valued from 0 to 1.

### Segmentation
When you specify `ImageType = Segmentation` in `ImageRequest`, you get image that gives you ground truth segmentation of the scene. AirSim assigns value 0 to 255 to each mesh available in environment. This value is than mapped to a specific color in [the pallet](../Unreal/Plugins/AirSim/Content/HUDAssets/seg_color_pallet.png). Currently, if you had like to assign specific value to specific map, you will need to [change code](../Unreal/Plugins/AirSim/Source/FlyingPawn.cpp#L28). We are planning to enable APIs for this in future.

## Collision API
The collision information can be obtained using `getCollisionInfo` API. This call returns a struct that has information not only whether collision occurred but also collision position, surface normal, penetration depth and so on.

## Example Code
A complete example of setting vehicle positions at random locations and orientations and then taking images can be found in [GenerateImageGenerator.hpp](../Examples/StereoImageGenerator.hpp). This example generates specified number of stereo images and ground truth disparity image and saving it to [pfm format](pfm.md).
