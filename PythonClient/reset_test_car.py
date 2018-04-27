from AirSimClient import *

# connect to the AirSim simulator 
client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
car_controls = CarControls()

# go forward
car_controls.throttle = 1
car_controls.steering = 1
client.setCarControls(car_controls)
print("Go Foward")
time.sleep(5)   # let car drive a bit

print("reset")
client.reset()
time.sleep(5)   # let car drive a bit

client.setCarControls(car_controls)
print("Go Foward")
time.sleep(5)   # let car drive a bit


