# This script expects object available in UE environment of type AAirSimCharater
# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim


import pprint
import os
import time

pp = pprint.PrettyPrinter(indent=4)

client = airsim.VehicleClient()
client.confirmConnection()

#airsim.wait_key('Press any key to set skin age to 0.9')
#client.simCharSetSkinAgeing(0.9)

#airsim.wait_key('Press any key to set skin color to 0.1')
#client.simCharSetSkinDarkness(0.1)

#airsim.wait_key('Press any key to set face expression')
#client.simCharSetFaceExpression("BlendShapeNode_Smile", 1);

#airsim.wait_key('Press any key to set bone pose')
jaw_pose = airsim.Pose()
#jaw_pose.position = airsim.Vector3r(0.002, 0.001, -0.003)
#jaw_pose.orientation = airsim.to_quaternion(0, 0, -.15)
client.simCharSetBonePose( "Jaw", jaw_pose);

airsim.wait_key('Press any key to turn head around')
for pitch in range(-15, 15, 3):
    for yaw in range(-30, 30, 2):
        q = airsim.to_quaternion(pitch/10.0, 0, yaw/10.0)
        client.simCharSetHeadRotation(q)
        time.sleep(0.1)

airsim.wait_key('Press any key to get images')
for x in range(3): # do few times
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.DepthVis),
        airsim.ImageRequest("0", airsim.ImageType.Segmentation),
        airsim.ImageRequest("0", airsim.ImageType.Scene),
        airsim.ImageRequest("0", airsim.ImageType.SurfaceNormals)])

    for i, response in enumerate(responses):
        if response.pixels_as_float:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
            airsim.write_pfm(os.path.normpath('/temp/cv_mode_' + str(x) + "_" + str(i) + '.pfm'), airsim.get_pfm_array(response))
        else:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
            airsim.write_file(os.path.normpath('/temp/cv_mode_' + str(x) + "_" + str(i) + '.png'), response.image_data_uint8)

    z = x * -20 - 5 # some random number
    client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(z, z, z), airsim.to_quaternion(x / 3.0, 0, x / 3.0)), True)

    time.sleep(3)

client.reset()