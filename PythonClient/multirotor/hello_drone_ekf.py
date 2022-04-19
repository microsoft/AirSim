# ready to run example: PythonClient/multirotor/hello_drone_ekf.py
import airsim
import time

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

client.startRecording()
client.resetVehicleApi()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
client.moveToPositionAsync(-10, 10, -10, 5).join()

# sleep
time.sleep(10)

# land
client.moveToPositionAsync(0, 0, -2, 5).join()
client.landAsync().join()

# stop
time.sleep(5)
client.stopRecording()
client.armDisarm(False)
client.enableApiControl(False)
