# Image APIs

## Getting Single Image
Here's a sample code to get a single image:

```
int getOneImage() 
{
    using namespace std;
    using namespace msr::airlib;
    
    msr::airlib::RpcLibClient client;

    vector<uint8_t> png_image = client.simGetImage(0, DroneControlBase::ImageType::Depth);
    //do something with images
}
```

Returned image is always in png format. To get uncompressed and other format please see next section.

## Getting Stereo/Multiple Images at Once

The `simGetImages` API whichis slighly more complex to use than `simGetImage` API, for example, you can get left camera view, right camera view and depth image from left camera - all at once! 

```
int getStereoAndDepthImages() 
{
    using namespace std;
    using namespace msr::airlib;
    
    typedef DroneControllerBase::ImageRequest ImageRequest;
    typedef VehicleCameraBase::ImageResponse ImageResponse;
    typedef VehicleCameraBase::ImageType_ ImageType_;

    msr::airlib::RpcLibClient client;

    //get right, left and depth images. First two as png, second as float16.
    vector<ImageRequest> request = { 
        ImageRequest(0, ImageType_::Scene), 
        ImageRequest(1, ImageType_::Scene),        
        ImageRequest(1, ImageType_::Depth, true) 
    };
    const vector<ImageResponse>& response = client.simGetImages(request);
    //do something with response which contains image data, pose, timestamp etc
}
```
Forready to run sample code please see [sample code in HelloDrone project](https://github.com/Microsoft/AirSim/blob/master/HelloDrone/main.cpp). 

We also have [complete code](https://github.com/Microsoft/AirSim/blob/master/Examples/StereoImageGenerator.hpp) that generates specified number of stereo images and ground truth depth with normalization to camera plan, computation of disparity image and saving it to pfm format.

Unlike `simGetImage`, the `simGetImages` API also allows you to get uncompressed images as well as floating point single channel images (instead of 3 channel (RGB), each 8 bit).

You can also use Python to get images. For sample code please see [PythonClient project](https://github.com/Microsoft/AirSim/tree/master/PythonClient) and [Python example doc](python.md).

## "Computer Vision" Mode

You can use AirSim in so-called "Computer Vision" mode. In this mode, physics engine is disabled and there is no flight controller active. This means when you start AirSim, vehicle would just hang in air. However you can move around using keyboard (use F1 to see help on keys). You can press Record button to continuously generate images. Or you can call APIs to move around and take images.

To active this mode, simply go to settings.json that you can find in your Documents\AirSim folder (or ~/Documents/AirSim on Linux) and make sure following values exist at root level:

```
{
  "DefaultVehicleConfig": "SimpleFlight",
  "UsageScenario": "ComputerVision"
}
```

## How to Set Position and Orientation (Pose)?

To move around the environment using APIs you can use `simSetPose` API. This API takes position and orientation and sets that on the vehicle. If you don't want to change position (or orientation) then set components of position (or orientation) to floating point nan values.

## Changing Resolution and Camera Parameters
To change resolution, FOV etc, you can use [settings.json](settings.md). For example, below is the complete content of settings.json that sets parameters for scene capture and uses "Computer Vision" mode described above. If you ommit any setting then below default values will be used. For more information see [settings doc](settings.md). If you are using stereo camera, currently the distance between left and right is fixed at 25 cm.

```
{
  "SceneCaptureSettings" : {
    "Width": 256,
	"Height": 144,
    "FOV_Degrees": 90,
    "AutoExposureSpeed": 100,
    "MotionBlurAmount": 0
  },
  "DefaultVehicleConfig": "SimpleFlight",
  "UsageScenario": "ComputerVision"
}
```

## What Does Values in Depth Image Mean?
Unreal generates the ground truth *perspective* depth image. The value of each pixel is a depth value from 0 to 2^24-1. However as this range is too large, the depth map would end up looking like all white. So we clip the depth at 10,000 cm and then normalize it from 0.0 to 1.0. To get the planner depth you need to do additional processing (for example, [see this function](https://github.com/Microsoft/AirSim/blob/master/Examples/StereoImageGenerator.hpp#L200)).

## Collision API
The collision information can be obtained using `getCollisionInfo` API. This call returns a struct that has information not only whether collision occured but also collision position, surface normal, penetration depth and so on.

## Complete Example
A complete example of setting vehicle positions at random locations and orientations and then taking images can be found in [GenerateImageGenerator.hpp]((https://github.com/Microsoft/AirSim/blob/master/Examples/StereoImageGenerator.hpp). This example generates specified number of stereo images and ground truth depth with normalization to camera plan, computation of disparity image and saving it to pfm format in this mode.
