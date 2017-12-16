# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *
import pprint

pp = pprint.PrettyPrinter(indent=4)

client = MultirotorClient()
client.confirmConnection()

for x in range(3): # do few times
    z = x * -20 - 5 # some random number
    client.simSetPose(Pose(Vector3r(z, z, z), AirSimClientBase.toQuaternion(x / 3.0, 0, x / 3.0)), True)

    responses = client.simGetImages([
        ImageRequest(0, AirSimImageType.DepthVis),
        ImageRequest(1, AirSimImageType.DepthPerspective, True),
        ImageRequest(2, AirSimImageType.Segmentation),
        ImageRequest(3, AirSimImageType.Scene),
        ImageRequest(4, AirSimImageType.DisparityNormalized),
        ImageRequest(4, AirSimImageType.SurfaceNormals)])

    for i, response in enumerate(responses):
        if response.pixels_as_float:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
            AirSimClientBase.write_pfm(os.path.normpath('/temp/cv_mode_' + str(x) + "_" + str(i) + '.pfm'), AirSimClientBase.getPfmArray(response))
        else:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
            AirSimClientBase.write_file(os.path.normpath('/temp/cv_mode_' + str(x) + "_" + str(i) + '.png'), response.image_data_uint8)

    pose = client.simGetPose()
    pp.pprint(pose)

    time.sleep(3)

# currently reset() doesn't work in CV mode. Below is the workaround
client.simSetPose(Pose(Vector3r(0, 0, 0), AirSimClientBase.toQuaternion(0, 0, 0)), True)