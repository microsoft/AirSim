# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *
import pprint
import tempfile

pp = pprint.PrettyPrinter(indent=4)

client = MultirotorClient()
client.confirmConnection()

AirSimClientBase.wait_key('Press any key to get camera parameters')
for camera_id in range(2):
    camera_info = client.getCameraInfo(camera_id)
    print("CameraInfo %d: %s" % (camera_id, pp.pprint(camera_info)))

AirSimClientBase.wait_key('Press any key to get images')
tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    for n in range(3):
        os.makedirs(os.path.join(tmp_dir, str(n)))
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for x in range(50): # do few times
    #xn = 1 + x*5  # some random number
    client.simSetPose(Pose(Vector3r(x, 0, -2), AirSimClientBase.toQuaternion(0, 0, 0)), True)
    time.sleep(0.1)

    responses = client.simGetImages([
        ImageRequest(0, AirSimImageType.Scene),
        ImageRequest(1, AirSimImageType.Scene),
        ImageRequest(2, AirSimImageType.Scene)])

    for i, response in enumerate(responses):
        if response.pixels_as_float:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_float), pprint.pformat(response.camera_position)))
            AirSimClientBase.write_pfm(os.path.normpath(os.path.join(tmp_dir, str(x) + "_" + str(i) + '.pfm')), AirSimClientBase.getPfmArray(response))
        else:
            print("Type %d, size %d, pos %s" % (response.image_type, len(response.image_data_uint8), pprint.pformat(response.camera_position)))
            AirSimClientBase.write_file(os.path.normpath(os.path.join(tmp_dir, str(i), str(x) + "_" + str(i) + '.png')), response.image_data_uint8)

    pose = client.simGetPose()
    pp.pprint(pose)

    time.sleep(3)

# currently reset() doesn't work in CV mode. Below is the workaround
client.simSetPose(Pose(Vector3r(0, 0, 0), AirSimClientBase.toQuaternion(0, 0, 0)), True)
