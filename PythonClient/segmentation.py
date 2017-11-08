# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

from AirSimClient import *

client = CarClient()
client.confirmConnection()

AirSimClientBase.wait_key('Press any key to set all object IDs to 0')
found = client.simSetSegmentationObjectID("[\w]*", 0, True);
print("Done: %r" % (found))

AirSimClientBase.wait_key('Press any key to set all object IDs to 0 hard way')
alphabet = ['a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z']
for letter in alphabet:
    found = client.simSetSegmentationObjectID(letter+"[\w]*", 0, True);
print("Done: %r" % (found))

AirSimClientBase.wait_key('Press any key to change one ground object ID')
found = client.simSetSegmentationObjectID("Ground", 20);
print("Done: %r" % (found))

AirSimClientBase.wait_key('Press any key to change all ground object ID')
found = client.simSetSegmentationObjectID("ground[\w]*", 22, True);
print("Done: %r" % (found))

found = client.simSetSegmentationObjectID("tree[\w]*", 2, True);
found = client.simSetSegmentationObjectID("landscape[\w]*", 200, True);
print("Done: %r" % (found))





