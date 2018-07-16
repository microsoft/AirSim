# Upgrading Settings

The settings schema in AirSim 1.2 is changed for more flexibility and cleaner interface. If you have older [settings.json](settings.md) file then you can either delete it and restart AirSim or use this guide to make manual upgrade.

## Quicker Way
We recommend simply deleting the [settings.json](settings.md) and add back the settings you need.
Please see [the doc](settings.md) for complete information on available settings.

## Changes

### UsageScenario
Previously we used `UsageScenario` to specify the `ComputerVision` mode. Now we use `"SimMode": "ComputerVision"` instead.

### CameraDefaults and Changing Camera Settings
Previously we had `CaptureSettings` and `NoiseSettings` in root. Now these are combined in new `CameraDefaults` element. The [schema for this element](settings.md#camera_settings) is later used to configure cameras on vehicle.

### Gimbal
The [Gimbal element](settings.md#Gimbal) (instead of old Gimble element) is now moved out of `CaptureSettings`.

### CameraID to CameraName
All settings now reference cameras by [name](image_apis.md#available_cameras) instead of ID.

### Using PX4
The new Vehicles element allows to specify which vehicles to create. To use PX4, please see [this section](settings.md#using_px4).

### AdditionalCameras
The old `AdditionalCameras` setting is now replaced by [Cameras element](settings.md#Common_Vehicle_Setting) within vehicle setting.

