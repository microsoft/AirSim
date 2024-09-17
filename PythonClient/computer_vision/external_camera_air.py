import setup_path
import airsim
import os
import tempfile
import numpy as np
import cv2
import time

"""
A simple script to test all the camera APIs. Change the camera name and whether it's an external camera

Example Settings for external camera -

{
    "SettingsVersion": 1.2,
    "SimMode": "Car",
    "ExternalCameras": {
        "fixed1": {
            "X": 0, "Y": 0, "Z": -5,
            "Pitch": -90, "Roll": 0, "Yaw": 0
        }
    }
}
"""

# Just change the below to test different cameras easily!
CAM_NAME = "fixed1"
CAM_NAME2 = "fixed2"
IS_EXTERNAL_CAM = True


client = airsim.VehicleClient()
client.confirmConnection()

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_cv_mode")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

print(f"Camera: {CAM_NAME}, External = {IS_EXTERNAL_CAM}")

# Test Camera info
cam_info = client.simGetCameraInfo(CAM_NAME, external=IS_EXTERNAL_CAM)
print(cam_info)

# Test Image APIs
airsim.wait_key('Press any key to get images')

#requests = [airsim.ImageRequest(CAM_NAME, airsim.ImageType.Scene),
#            airsim.ImageRequest(CAM_NAME, airsim.ImageType.DepthPlanar),
#            airsim.ImageRequest(CAM_NAME, airsim.ImageType.DepthVis),
#            airsim.ImageRequest(CAM_NAME, airsim.ImageType.Segmentation),
#            airsim.ImageRequest(CAM_NAME, airsim.ImageType.SurfaceNormals)]
requests = [airsim.ImageRequest(CAM_NAME, airsim.ImageType.Scene, False, False)]
requests2 = [airsim.ImageRequest(CAM_NAME2, airsim.ImageType.Scene, False, False)]


def save_images(responses, prefix = ""):
    for i, response in enumerate(responses):
        filename = os.path.join(tmp_dir, prefix + "_" + str(i))

        if response.pixels_as_float:
            print(f"Type {response.image_type}, size {len(response.image_data_float)}, pos {response.camera_position}")
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        else:
            print(f"Type {response.image_type}, size {len(response.image_data_uint8)}, pos {response.camera_position}")
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
cv2.namedWindow('war1')
cv2.namedWindow('war2')

for i in range(0,2400):
    responses = client.simGetImages(requests, external=IS_EXTERNAL_CAM)
    response = responses[0]
    img1d = np.frombuffer(response.image_data_uint8, dtype = np.uint8) 
    img_rgb = img1d.reshape(response.height, response.width,3)
    responses2 = client.simGetImages(requests2, external=IS_EXTERNAL_CAM)
    response2 = responses2[0]
    img1d2 = np.frombuffer(response2.image_data_uint8, dtype = np.uint8) 
    img_rgb2 = img1d2.reshape(response2.height, response2.width,3)
    cv2.imshow("war1", img_rgb)
    cv2.imshow("war2", img_rgb2)
    cv2.waitKey(1)
    time.sleep(0.05)



# Test FoV API
