# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *
import pprint

pp = pprint.PrettyPrinter(indent=4)

# or use CarClient()
client = CarClient()
client.confirmConnection()

for x in range(3): # do few times
    z = x * -20 - 5
    # Use MultirotorClient.toQuaternion(0, 0, x) to generate quaternion
    client.simSetPose(Pose(Vector3r(1, 1, z), Quaternionr(0, 0, 0, 1)))

    responses = client.simGetImages([
        ImageRequest(0, AirSimImageType.DepthVis),
        ImageRequest(1, AirSimImageType.DepthPerspective, True),
        ImageRequest(0, AirSimImageType.Segmentation),
        ImageRequest(0, AirSimImageType.Scene),
        ImageRequest(0, AirSimImageType.DisparityNormalized),
        ImageRequest(0, AirSimImageType.SurfaceNormals)])

    for i, response in enumerate(responses):
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            MultirotorClient.write_pfm(os.path.normpath('/temp/cv_mode_' + str(x) + "_" + str(i) + '.pfm'), MultirotorClient.getPfmArray(response))
        else:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            MultirotorClient.write_file(os.path.normpath('/temp/cv_mode_' + str(x) + "_" + str(i) + '.png'), response.image_data_uint8)

    pose = client.simGetPose()
    pp.pprint(pose)

    time.sleep(3)
