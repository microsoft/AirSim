import airsim
import pprint

def print_state(client):
    state = client.getMultirotorState()
    s = pprint.pformat(state)
    print("state: %s" % s)

    imu_data = client.getImuData()
    s = pprint.pformat(imu_data)
    print("imu_data: %s" % s)

    barometer_data = client.getBarometerData()
    s = pprint.pformat(barometer_data)
    print("barometer_data: %s" % s)

    magnetometer_data = client.getMagnetometerData()
    s = pprint.pformat(magnetometer_data)
    print("magnetometer_data: %s" % s)

    gps_data = client.getGpsData()
    s = pprint.pformat(gps_data)
    print("gps_data: %s" % s)

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print_state(client)
print('Takeoff')
client.takeoffAsync().join()

while True:
    print_state(client)
    print('Go to (-10, 10, -10) at 5 m/s')
    client.moveToPositionAsync(-10, 10, -10, 5).join()
    client.hoverAsync().join()
    print_state(client)
    print('Go to (0, 10, 0) at 5 m/s')
    client.moveToPositionAsync(0, 10, 0, 5).join()
