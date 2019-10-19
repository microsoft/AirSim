# How to Use Car in AirSim

By default AirSim prompts user for which vehicle to use. You can easily change this by setting [SimMode](settings.md#SimMode). For example, if you want to use car instead then just set the SimMode in your [settings.json](settings.md) which you can find in your `~/Documents/AirSim` folder, like this:

```
{
  "SettingsVersion": 1.2,
  "SimMode": "Car"
}
```

Now when you restart AirSim, you should see the car spawned automatically.

## Manual Driving 
Please use the keyboard arrow keys to drive manually. Spacebar for the handbrake. In manual drive mode, gears are set in "auto".

## Using APIs
You can control the car, get state and images by calling APIs in variety of client languages including C++ and Python. Please see [APIs doc](apis.md) for more details.

## Changing Views
By default camera will chase the car from the back. You can get the FPV view by pressing `F` key and switch back to chasing from back view by pressing `/` key. More keyboard shortcuts can be seen by pressing F1.

## Cameras
By default car is installed with 5 cameras: center, left and right, driver and reverse. You can chose the images from these camera by specifying [the name](image_apis.md#available_cameras).
