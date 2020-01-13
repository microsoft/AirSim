# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim

import pprint
import os
import time
import tempfile

pp = pprint.PrettyPrinter(indent=4)


client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.simSetCameraOrientation("0", airsim.to_quaternion(0.261799 *3, 0, 0)); #radians


landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    print("already flying...")
    client.hoverAsync().join()


tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print("path: %s" % tmp_dir)

for x in range(3): # do few times
    z = x * -20 - 5 # some random number
 #   client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(z, z, z), airsim.to_quaternion(x / 3.0, 0, x / 3.0)), True)

    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.DepthVis),
        airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True),
        airsim.ImageRequest("2", airsim.ImageType.Segmentation),
        airsim.ImageRequest("3", airsim.ImageType.Scene),
        airsim.ImageRequest("4", airsim.ImageType.DisparityNormalized),
        airsim.ImageRequest("4", airsim.ImageType.SurfaceNormals)])

    print("path: %s" % os.path.normpath('../temp/'))
    for i, response in enumerate(responses):
        
        if response.pixels_as_float:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
            airsim.write_pfm(os.path.normpath('../temp/cv_mode_' + str(x) + "_" + str(i) + '.pfm'), airsim.get_pfm_array(response))
        else:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
            #airsim.write_file(os.path.normpath('../temp/cv_mode_' + str(x) + "_" + str(i) + '.png'), response.image_data_uint8)
            filename = os.path.join(tmp_dir, str('cv_mode_' + str(x) + "_" + str(i)))
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    pose = client.simGetVehiclePose()
    pp.pprint(pose)

    time.sleep(3)

# currently reset() doesn't work in CV mode. Below is the workaround