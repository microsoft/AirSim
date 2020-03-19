import setup_path
import airsim
from datetime import datetime

'''
Example file showing how to use and disable a high-resolution camera
Useful when it's used only occasionaly, and without disabling the camera affects the simulator performance

Settings used-
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles" : {
        "Drone" : {
            "VehicleType" : "SimpleFlight",
            "DefaultVehicleState" : "Armed",
            "AutoCreate" : true,
            "Cameras" : {
                "high_res": {
                    "CaptureSettings" : [
                        {
                            "ImageType" : 0,
                            "Width" : 4320,
                            "Height" : 2160
                        }
                    ],
                    "X": 0.50, "Y": 0.00, "Z": 0.10,
                    "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
                },
                "low_res": {
                    "CaptureSettings" : [
                        {
                            "ImageType" : 0,
                            "Width" : 256,
                            "Height" : 144
                        }
                    ],
                    "X": 0.50, "Y": 0.00, "Z": 0.10,
                    "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
                }
            }
        }
    }
}

'''

client = airsim.MultirotorClient()
client.confirmConnection()
framecounter = 1

prevtimestamp = datetime.now()

while(framecounter <= 1500):
    if framecounter%500 == 0:
        client.simGetImages([airsim.ImageRequest("high_res", airsim.ImageType.Scene, False, False)])
        # Disable camera, simGetImages automatically enables the camera
        client.simDisableCamera("high_res", airsim.ImageType.Scene)
        print("High resolution image captured.")

    if framecounter%30 == 0:
        now = datetime.now()
        print("Time spent for 30 frames: " + str(now-prevtimestamp))
        prevtimestamp = now

    client.simGetImages([airsim.ImageRequest("low_res", airsim.ImageType.Scene, False, False)])
    framecounter += 1
