# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *

client = CarClient()
client.confirmConnection()

found = client.simSetSegmentationObjectID("Ground", 20);
print("Done: %r" % (found))

found = client.simSetSegmentationObjectID("Ground_2", 22);
print("Done: %r" % (found))

found = client.simSetSegmentationObjectID("Ground_3", 23);
print("Done: %r" % (found))

found = client.simSetSegmentationObjectID("Ground_4", 24);
print("Done: %r" % (found))

found = client.simSetSegmentationObjectID("Ground_5", 25);
print("Done: %r" % (found))

found = client.simSetSegmentationObjectID("Ground_6", 26);
print("Done: %r" % (found))

found = client.simSetSegmentationObjectID("Ground_7", 27);
print("Done: %r" % (found))



