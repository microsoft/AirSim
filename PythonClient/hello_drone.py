from PythonClient import *

# connect to the AirSim simulator 
client = AirSimClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

AirSimClient.wait_key('Press any key to takeoff')
client.takeoff()

AirSimClient.wait_key('Press any key to take images')
responses = client.simGetImages([
    ImageRequest(0, AirSimImageType.DepthVis), 
    ImageRequest(1, AirSimImageType.DepthMeters, True)])
print('Retrieved images: %d', len(responses))

for response in responses:
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        AirSimClient.write_pfm(os.path.normpath('/temp/py1.pfm'), AirSimClient.getPfmArray(response))
    else:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        AirSimClient.write_file(os.path.normpath('/temp/py1.png'), response.image_data_uint8)
