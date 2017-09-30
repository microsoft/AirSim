from PythonClient import *

# connect to the AirSim simulator 
client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

MultirotorClient.wait_key('Press any key to takeoff')
client.takeoff()

MultirotorClient.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPosition(-10, 10, -10, 5)

MultirotorClient.wait_key('Press any key to take images')
responses = client.simGetImages([
    ImageRequest(0, AirSimImageType.DepthVis), 
    ImageRequest(1, AirSimImageType.DepthPlanner, True)])
print('Retrieved images: %d', len(responses))

for response in responses:
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        MultirotorClient.write_pfm(os.path.normpath('/temp/py1.pfm'), MultirotorClient.getPfmArray(response))
    else:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        MultirotorClient.write_file(os.path.normpath('/temp/py1.png'), MultirotorClient.image_data_uint8)
