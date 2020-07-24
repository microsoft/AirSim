# Please add "EnableTrace": true to your setting.json as shown below

# {
#   "SettingsVersion": 1.2,
#   "SimMode": "Multirotor",
#   "Vehicles": {
#       "Drone": {
#           "VehicleType": "SimpleFlight",
#           "EnableTrace": true
#         }
#     }
# }

import setup_path
import airsim
import time

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()
client.hoverAsync().join()

vehicleControl = client.moveByVelocityAsync(1, 1, 0, 12)

client.simSetTraceLine([1.0, 0.0, 0.0, 1.0], 5)
time.sleep(2)
client.simSetTraceLine([0.0, 1.0, 0.0, 0.8], 10)
time.sleep(2)
client.simSetTraceLine([0.0, 0.0, 1.0, 0.6], 20)
time.sleep(2)
client.simSetTraceLine([1.0, 1.0, 0.0, 0.4], 30)
time.sleep(2)
client.simSetTraceLine([0.0, 1.0, 1.0, 0.2], 40)
time.sleep(2)
client.simSetTraceLine([1.0, 0.0, 1.0, 0.1], 50)
time.sleep(2)

vehicleControl.join()

client.armDisarm(False)
client.takeoffAsync().join()
client.enableApiControl(False)
