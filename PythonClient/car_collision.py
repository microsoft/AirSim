from AirSimClient import *
import pprint

# connect to the AirSim simulator 
client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = CarControls()

client.reset()

# go forward
car_controls.throttle = 0.5
car_controls.steering = 0
client.setCarControls(car_controls)

while True:
    # get state of the car
    car_state = client.getCarState()
    print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))

    collision_info = client.getCollisionInfo()

    if collision_info.has_collided:
        print("Collision at pos %s, normal %s, impact pt %s, penetration %f, name %s, obj id %d" % (
            pprint.pformat(collision_info.position), 
            pprint.pformat(collision_info.normal), 
            pprint.pformat(collision_info.impact_point), 
            collision_info.penetration_depth, collision_info.object_name, collision_info.object_id))
        break

    time.sleep(0.1)

client.enableApiControl(False)
