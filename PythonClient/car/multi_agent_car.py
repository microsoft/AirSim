import airsim
import cv2
import numpy as np
import os
import setup_path 
import time

# Use below in settings.json with blocks environment
"""
{
	"SettingsVersion": 1.2,
	"SimMode": "Car",
	
	"Vehicles": {
		"Car1": {
		  "VehicleType": "PhysXCar",
		  "X": 4, "Y": 0, "Z": -2
		},
		"Car2": {
		  "VehicleType": "PhysXCar",
		  "X": -4, "Y": 0, "Z": -2
		}

    }
}
"""

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True, "Car1")
client.enableApiControl(True, "Car2")

car_controls1 = airsim.CarControls()
car_controls2 = airsim.CarControls()


for idx in range(3):
    # get state of the car
    car_state1 = client.getCarState("Car1")
    print("Car1: Speed %d, Gear %d" % (car_state1.speed, car_state1.gear))
    car_state2 = client.getCarState("Car2")
    print("Car1: Speed %d, Gear %d" % (car_state2.speed, car_state2.gear))

    # go forward
    car_controls1.throttle = 0.5
    car_controls1.steering = 0.5
    client.setCarControls(car_controls1, "Car1")
    print("Car1: Go Forward")

    car_controls2.throttle = 0.5
    car_controls2.steering = -0.5
    client.setCarControls(car_controls2, "Car2")
    print("Car2: Go Forward")
    time.sleep(3)   # let car drive a bit


    # go reverse
    car_controls1.throttle = -0.5
    car_controls1.is_manual_gear = True;
    car_controls1.manual_gear = -1
    car_controls1.steering = -0.5
    client.setCarControls(car_controls1, "Car1")
    print("Car1: Go reverse, steer right")
    car_controls1.is_manual_gear = False; # change back gear to auto
    car_controls1.manual_gear = 0  

    car_controls2.throttle = -0.5
    car_controls2.is_manual_gear = True;
    car_controls2.manual_gear = -1
    car_controls2.steering = 0.5
    client.setCarControls(car_controls2, "Car2")
    print("Car2: Go reverse, steer right")
    car_controls2.is_manual_gear = False; # change back gear to auto
    car_controls2.manual_gear = 0  
    time.sleep(3)   # let car drive a bit


    # apply breaks
    car_controls1.brake = 1
    client.setCarControls(car_controls1, "Car1")
    print("Car1: Apply break")
    car_controls1.brake = 0 #remove break

    car_controls2.brake = 1
    client.setCarControls(car_controls2, "Car2")
    print("Car2: Apply break")
    car_controls2.brake = 0 #remove break
    time.sleep(3)   # let car drive a bit
    
    # get camera images from the car
    responses1 = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
        airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], "Car1")  #scene vision image in uncompressed RGB array
    print('Car1: Retrieved images: %d' % (len(responses1)))
    responses2 = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Segmentation),  #depth visualization image
        airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], "Car2")  #scene vision image in uncompressed RGB array
    print('Car2: Retrieved images: %d' % (len(responses2)))

    for response in responses1 + responses2:
        filename = 'c:/temp/car_multi_py' + str(idx)

        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        elif response.compress: #png format
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else: #uncompressed array
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
            img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 3 channel image array H X W X 3
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

#restore to original state
client.reset()

client.enableApiControl(False)


            
