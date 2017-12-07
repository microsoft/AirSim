from AirSimClient import *
import pprint


client = MultirotorClient()
client.confirmConnection()

pose = client.simGetObjectPose("TemplateCube_Rounded_17");
print("Position: %s, Orientation: %s" % (pprint.pformat(pose.position),
    pprint.pformat(pose.orientation)))
