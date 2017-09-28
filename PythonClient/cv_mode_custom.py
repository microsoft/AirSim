# In settings.json first activate computer vision mode:
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from PythonClient import *

client = AirSimClient()
client.confirmConnection()

z = -5
y = 0
for x in range(20):
    z -= 1
    y -= 0.1

    # you can also use AirSimClient.toQuaternion(0, 0, x) to generate quaternion
    # client.simSetPose(Vector3r(1, 1, z), Quaternionr(0, 0, 60, 1))
    client.simSetPose(Vector3r(1,1,z), AirSimClient.toQuaternion(y,0,0))

    responses = client.simGetImages([ImageRequest(0, AirSimImageType.Segmentation)])

    for i, response in enumerate(responses):
        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            AirSimClient.write_pfm(os.path.normpath('./temp/cv_mode_' + str(x) + "_" + str(i) + '.pfm'), AirSimClient.getPfmArray(response))
        else:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            AirSimClient.write_file(os.path.normpath('./temp/cv_mode_' + str(x) + "_" + str(i) + '.png'), response.image_data_uint8)
