import setup_path
import airsim
import time
import sys
import threading


def runSingleCar(id: int):
    client = airsim.CarClient()
    client.confirmConnection()

    vehicle_name = f"Car_{id}"
    pose = airsim.Pose(airsim.Vector3r(0, 7*id, 0),
                       airsim.Quaternionr(0, 0, 0, 0))

    print(f"Creating {vehicle_name}")
    success = client.simAddVehicle(vehicle_name, "Physxcar", pose)

    if not success:
        print(f"Falied to create {vehicle_name}")
        return

    # Sleep for some time to wait for other vehicles to be created
    time.sleep(1)

    # driveCar(vehicle_name, client)
    print(f"Driving {vehicle_name} for a few secs...")
    client.enableApiControl(True, vehicle_name)

    car_controls = airsim.CarControls()

    # go forward
    car_controls.throttle = 0.5
    car_controls.steering = 0
    client.setCarControls(car_controls, vehicle_name)
    time.sleep(3)   # let car drive a bit

    # Go forward + steer right
    car_controls.throttle = 0.5
    car_controls.steering = 1
    client.setCarControls(car_controls, vehicle_name)
    time.sleep(3)

    # go reverse
    car_controls.throttle = -0.5
    car_controls.is_manual_gear = True
    car_controls.manual_gear = -1
    car_controls.steering = 0
    client.setCarControls(car_controls, vehicle_name)
    time.sleep(3)
    car_controls.is_manual_gear = False  # change back gear to auto
    car_controls.manual_gear = 0

    # apply brakes
    car_controls.brake = 1
    client.setCarControls(car_controls, vehicle_name)
    time.sleep(3)


if __name__ == "__main__":
    num_vehicles = 3

    if len(sys.argv) == 2:
        num_vehicles = int(sys.argv[1])

    print(f"Creating {num_vehicles} vehicles")

    threads = []
    for id in range(num_vehicles, 0, -1):
        t = threading.Thread(target=runSingleCar, args=(id,))
        threads.append(t)
        t.start()

    for t in threads:
        t.join()
