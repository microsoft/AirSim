# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim

import pprint

client = airsim.VehicleClient()
client.confirmConnection()

#Go to object in Unreal Editor, click on it and then look for Tags property. 
#Add a tag "MyObject" (without quotes), save and the query using following line
#see more about tags here: https://answers.unrealengine.com/questions/543807/whats-the-difference-between-tag-and-tag.html
pose = client.simGetObjectPose("MyObject");
print("Position: %s, Orientation: %s" % (pprint.pformat(pose.position),
    pprint.pformat(pose.orientation)))
