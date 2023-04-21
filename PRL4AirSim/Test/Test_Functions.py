import time

import airsim
import numpy as np

def convert_pos_UE_to_AS(origin_UE : np.array, pos_UE : np.array):
    pos = np.zeros(3, dtype=np.float)
    pos[0] = pos_UE[0] - origin_UE[0]
    pos[1] = pos_UE[1] - origin_UE[1]
    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100

droneName = "Drone0"
origin_UE = np.array([0.0, 0.0, 910.0])
areans_train_long = np.array([
    # Using larger environment
    # [Utils.convert_pos_UE_to_AS(self.origin_UE, np.array([41156.0, 20459.0, 1000.0])), Utils.convert_pos_UE_to_AS(self.origin_UE, np.array([56206.0, 21019.0, 1000.0]))]
    # Using smaller environment
    [convert_pos_UE_to_AS(origin_UE, np.array([8430.0, -6760.0, 1000.0])),
     convert_pos_UE_to_AS(origin_UE, np.array([14060.0, -6760.0, 1000.0]))]
])


client = airsim.MultirotorClient(ip="127.0.0.1", port=29001)
client.confirmConnection()
client.reset()

client.armDisarm(True, droneName)
client.enableApiControl(True, droneName)
client.takeoffAsync(vehicle_name=droneName)
time.sleep(2)

client.client.call_async("resetVehicle", droneName,
                                    airsim.Pose(airsim.Vector3r(areans_train_long[0][0][0],
                                                                areans_train_long[0][0][1],
                                                                areans_train_long[0][0][2]),
                                                airsim.Quaternionr(0.0, 0.0, 0.0, 0.0)))
