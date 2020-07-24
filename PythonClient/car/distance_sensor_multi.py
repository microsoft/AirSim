import airsim
import time

'''
An example script showing usage of Distance sensor to measure distance between 2 Car vehicles
Settings -

{
  "SettingsVersion": 1.2,
  "SimMode": "Car",
  "Vehicles": {
    "Car1": {
      "VehicleType": "PhysXCar",
      "AutoCreate": true,
      "Sensors": {
        "Distance": {
            "SensorType": 5,
            "Enabled" : true,
            "DrawDebugPoints": true
        }
      }
    },
    "Car2": {
        "VehicleType": "PhysXCar",
        "AutoCreate": true,
        "X": 10, "Y": 0, "Z": 0,
        "Sensors": {
            "Distance": {
                "SensorType": 5,
                "Enabled" : true,
                "DrawDebugPoints": true
            }
        }
    }
  }
}

Car2 is placed in front of Car 1

'''

client = airsim.CarClient()
client.confirmConnection()

while True:
    data_car1 = client.getDistanceSensorData(vehicle_name="Car1")
    data_car2 = client.getDistanceSensorData(vehicle_name="Car2")
    print(f"Distance sensor data: Car1: {data_car1.distance}, Car2: {data_car2.distance}")
    time.sleep(1.0)