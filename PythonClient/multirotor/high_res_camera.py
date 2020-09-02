import airsim
from datetime import datetime

'''
Simple script with settings to create a high-resolution camera, and fetching it

Settings used-
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles" : {
        "Drone1" : {
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

client = airsim.VehicleClient()
client.confirmConnection()
framecounter = 1

prevtimestamp = datetime.now()

while(framecounter <= 500):
    if framecounter%150 == 0:
        client.simGetImages([airsim.ImageRequest("high_res", airsim.ImageType.Scene, False, False)])
        print("High resolution image captured.")

    if framecounter%30 == 0:
        now = datetime.now()
        print(f"Time spent for 30 frames: {now-prevtimestamp}")
        prevtimestamp = now

    client.simGetImages([airsim.ImageRequest("low_res", airsim.ImageType.Scene, False, False)])
    framecounter += 1
