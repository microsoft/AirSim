from AirSimClient import *
import pprint


client = MultirotorClient()
client.confirmConnection()

#Go to object in Unreal Editor, click on it and then look for Tags property. 
#Add a tag "MyObject" (without quotes), save and the query using following line
#see more about tags here: https://answers.unrealengine.com/questions/543807/whats-the-difference-between-tag-and-tag.html
pose = client.simGetObjectPose("MyObject");
print("Position: %s, Orientation: %s" % (pprint.pformat(pose.position),
    pprint.pformat(pose.orientation)))
