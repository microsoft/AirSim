from AirSimClient import *

# connect to the AirSim simulator 
client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = CarControls()

for idx in xrange(3000):
    # get state of the car
    car_state = client.getCarState()
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))

    # go forward
    car_controls.throttle = 0.5
    car_controls.steering = 0
    client.setCarControls(car_controls)
    time.sleep(0.1)   # let car drive a bit

    # Go forward + steer right
    car_controls.throttle = 0.5
    car_controls.steering = 1
    client.setCarControls(car_controls)
    time.sleep(0.1)   # let car drive a bit

    # go reverse
    car_controls.throttle = -0.5
    car_controls.is_manual_gear = True;
    car_controls.manual_gear = -1
    car_controls.steering = 0
    client.setCarControls(car_controls)
    time.sleep(0.1)   # let car drive a bit
    car_controls.is_manual_gear = False; # change back gear to auto
    car_controls.manual_gear = 0  

    # apply breaks
    car_controls.brake = 1
    client.setCarControls(car_controls)
    time.sleep(0.1)   # let car drive a bit
    car_controls.brake = 0 #remove break
    
    #restore to original state
    client.reset()

client.enableApiControl(False)


            
