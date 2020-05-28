import setup_path
import airsim
from datetime import datetime

'''
Example file for benchmarking and observing the affect of a high-resolution camera

Settings used-
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles" : {
        "Drone" : {
            "VehicleType" : "SimpleFlight",
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
    if framecounter%50 == 0:
        now = datetime.now()
        print("Time spent for 50 frames: " + str(now-prevtimestamp))
        prevtimestamp = now

    if framecounter%250 == 0:
        client.simGetImages([airsim.ImageRequest("high_res", airsim.ImageType.Scene, False, False)])
        print("High resolution image captured.")

    client.simGetImages([airsim.ImageRequest("low_res", airsim.ImageType.Scene, False, False)])
    framecounter += 1
