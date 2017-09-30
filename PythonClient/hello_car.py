from PythonClient import *

# connect to the AirSim simulator 
client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = CarControls()

while True:
    # get state of the car
    car_state = client.getCarState()
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))

    # go forward
    car_controls.throttle = 0.5
    car_controls.steering = 0
    client.setCarControls(car_controls)
    print("Go Foward")
    time.sleep(3)   # let car drive a bit

    # Go forward + steer right
    car_controls.throttle = 0.5
    car_controls.steering = 1
    client.setCarControls(car_controls)
    print("Go Foward, steer right")
    time.sleep(3)   # let car drive a bit

    # go reverse, steer left
    car_controls.throttle = -0.5
    car_controls.is_manual_gear = True;
    car_controls.manual_gear = -1
    car_controls.steering = -1
    client.setCarControls(car_controls)
    print("Go reverse, steer right")
    time.sleep(3)   # let car drive a bit
    car_controls.is_manual_gear = False; # change back gear to auto
    car_controls.manual_gear = 0  

    # apply breaks
    car_controls.brake = 1
    client.setCarControls(car_controls)
    print("Apply break")
    time.sleep(3)   # let car drive a bit
    car_controls.brake = 0 #remove break
    
    # get camera images from the car
    responses = client.simGetImages([
        ImageRequest(0, AirSimImageType.DepthVis),
        ImageRequest(1, AirSimImageType.DepthPerspective, True)]) 
    print('Retrieved images: %d', len(responses))

    for response in responses:
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            CarClient.write_pfm(os.path.normpath('c:/temp/py1.pfm'), CarClient.getPfmArray(response))
        else:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            CarClient.write_file(os.path.normpath('c:/temp/py1.png'), response.image_data_uint8)

