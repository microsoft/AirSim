# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *

client = CarClient()
client.confirmConnection()

AirSimClientBase.wait_key('Press any key to change one ground object ID')
found = client.simSetSegmentationObjectID("Ground", 20);
print("Done: %r" % (found))

AirSimClientBase.wait_key('Press any key to change all ground object ID')
found = client.simSetSegmentationObjectID("ground[\w]*", 22, True);
print("Done: %r" % (found))



