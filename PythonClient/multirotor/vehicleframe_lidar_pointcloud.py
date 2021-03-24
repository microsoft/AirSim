# Python client example to get Lidar data from a drone, although this script works for any AirSim-supported vehicle
# This script is for Lidar sensors using 'VehicleInertialFrame' as DataFrame under settings.json
# Sample settings.json used for this script:
#    {
#        "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings_json.md",
#        "SettingsVersion": 1.2,
#        "SimMode": "Multirotor",
#         "Vehicles": {
#            "Drone1": {
#                "VehicleType": "SimpleFlight",
#                "AutoCreate": true,
#                "Sensors": {
#                    "LidarSensor1": { 
#                        "SensorType": 6,
#                        "Enabled" : true,
#                        ...
#                        "DataFrame": "VehicleInertialFrame"
#                    },
#                    "LidarSensor2": { 
#                        "SensorType": 6,
#                        "Enabled" : true,
#                        ...
#                        "DataFrame": "VehicleInertialFrame"
#                    }
#                }
#            }
#        }
#    }
import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy as np

# Makes the drone fly and get Lidar data
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.VehicleClient()
        self.client.confirmConnection()
        print('Connected!\n')

    def execute(self,vehicle_name,lidar_names):
        #self.client.reset()
        print('Scanning Has Started\n')
        print('Use Keyboard Interrupt \'CTRL + C\' to Stop Scanning\n')
        existing_data_cleared = False   #change to true to superimpose new scans onto existing .asc files
        try:
            while True:
                for lidar_name in lidar_names:
                    filename = f"{vehicle_name}_{lidar_name}_pointcloud.asc"
                    if not existing_data_cleared:
                        f = open(filename,'w')
                    else:
                        f = open(filename,'a')
                    lidarData = self.client.getLidarData(lidar_name=lidar_name,vehicle_name=vehicle_name)
                    xyz_flag = 0
                    for xyz_values in lidarData.point_cloud:
                        if (xyz_flag == 0):
                            x = xyz_values
                        elif (xyz_flag == 1):
                            y = xyz_values
                        else:
                            z = xyz_values
                        xyz_flag = xyz_flag + 1
                        if (xyz_flag == 3):
                            f.write("%f %f %f %d %d %d \n" % (x,y,z,255,255,0))
                            xyz_flag=0
                        else:
                            pass
                    f.close()
                existing_data_cleared = True
        except KeyboardInterrupt:
            f.close()
            pass
            #time.sleep(5)
            airsim.wait_key('Press any key to reset to original state')
            self.client.reset()
            print("Done!\n")

# main
if __name__ == "__main__":
    lidarTest = LidarTest()
    lidarTest.execute('Drone1',['LidarSensor1','LidarSensor2'])
