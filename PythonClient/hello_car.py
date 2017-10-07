from AirSimClient import *

# connect to the AirSim simulator 
client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = CarControls()

for idx in range(3):
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

    # go reverse
    car_controls.throttle = -0.5
    car_controls.is_manual_gear = True;
    car_controls.manual_gear = -1
    car_controls.steering = 0
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
        ImageRequest(0, AirSimImageType.DepthVis),  #depth visualiztion image
        ImageRequest(1, AirSimImageType.DepthPerspective, True), #depth in perspective projection
        ImageRequest(1, AirSimImageType.Scene), #scene vision image in png format
        ImageRequest(1, AirSimImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
    print('Retrieved images: %d', len(responses))

    for response in responses:
        filename = 'c:/temp/py' + str(idx)

        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            AirSimClientBase.write_pfm(os.path.normpath(filename + '.pfm'), AirSimClientBase.getPfmArray(response))
        elif response.compress: #png format
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            AirSimClientBase.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else: #uncompressed array
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
            img_rgba = img1d.reshape(response.height, response.width, 4) #reshape array to 4 channel image array H X W X 4
            img_rgba = np.flipud(img_rgba) #original image is fliped vertically
            img_rgba[:,:,1:2] = 100 #just for fun add little bit of green in all pixels
            AirSimClientBase.write_png(os.path.normpath(filename + '.greener.png'), img_rgba) #write to png 


#restore to original state
client.reset()

client.enableApiControl(False)


            
