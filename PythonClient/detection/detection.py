import setup_path 
import airsim
import cv2
import numpy as np 
import pprint

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# set detection radius in [cm]
client.simSetDetectionFilterRadius(200 * 100) 
# add desired object name to detect in wild card/regex format
client.simAddDetectionFilterMeshName("Cylinder*") 

# set camera name and image type to request images and detections
camera_name = "0"
image_type = airsim.ImageType.Scene

while True:
    rawImage = client.simGetImage(camera_name, image_type)
    if not rawImage:
        continue
    png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
    cars = client.simGetDetections(camera_name, image_type)
    if cars:
        for car in cars:
            s = pprint.pformat(car)
            print("Cylinder: %s" % s)

            cv2.rectangle(png,(car.topLeft_x,car.topLeft_y),(car.bottomRight_x,car.bottomRight_y),(255,0,0),2)

    
    cv2.imshow("AirSim", png)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows() 