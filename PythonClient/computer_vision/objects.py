# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/main/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim

import pprint

client = airsim.VehicleClient()
client.confirmConnection()

# objects can be named in two ways:
# 1. In UE Editor, select and object and change its name to something else. Note that you must *change* its name because
#    default name is auto-generated and varies from run-to-run.
# 2. OR you can do this: In UE Editor select the object and then go to "Actor" section, click down arrow to see "Tags" property and add a tag there.
#
# The simGetObjectPose and simSetObjectPose uses first object that has specified name OR tag.
# more info: https://answers.unrealengine.com/questions/543807/whats-the-difference-between-tag-and-tag.html
#            https://answers.unrealengine.com/revisions/790629.html

# below works in Blocks environment

#------------------------------------ Get current pose ------------------------------------------------

# search object by name: 
pose1 = client.simGetObjectPose("OrangeBall");
print("OrangeBall - Position: %s, Orientation: %s" % (pprint.pformat(pose1.position),
    pprint.pformat(pose1.orientation)))

# search another object by tag
pose2 = client.simGetObjectPose("PulsingCone");
print("PulsingCone - Position: %s, Orientation: %s" % (pprint.pformat(pose2.position),
    pprint.pformat(pose2.orientation)))

# search non-existent object
pose3 = client.simGetObjectPose("Non-Existent"); # should return nan pose
print("Non-Existent - Position: %s, Orientation: %s" % (pprint.pformat(pose3.position),
    pprint.pformat(pose3.orientation)))


#------------------------------------ Set new pose ------------------------------------------------

# here we move with teleport enabled so collisions are ignored
pose1.position = pose1.position + airsim.Vector3r(-2, -2, -2)
success = client.simSetObjectPose("OrangeBall", pose1, True);
airsim.wait_key("OrangeBall moved. Success: %i" % (success))

# here we move with teleport enabled so collisions are not ignored
pose2.position = pose2.position + airsim.Vector3r(3, 3, -2)
success = client.simSetObjectPose("PulsingCone", pose2, False);
airsim.wait_key("PulsingCone moved. Success: %i" % (success))

# move non-existent object
success = client.simSetObjectPose("Non-Existent", pose2); # should return nan pose
airsim.wait_key("Non-Existent moved. Success: %i" % (success))

#------------------------------------ Get new pose ------------------------------------------------


pose1 = client.simGetObjectPose("OrangeBall");
print("OrangeBall - Position: %s, Orientation: %s" % (pprint.pformat(pose1.position),
    pprint.pformat(pose1.orientation)))

# search another object by tag
pose2 = client.simGetObjectPose("PulsingCone");
print("PulsingCone - Position: %s, Orientation: %s" % (pprint.pformat(pose2.position),
    pprint.pformat(pose2.orientation)))

# search non-existent object
pose3 = client.simGetObjectPose("Non-Existent"); # should return nan pose
print("Non-Existent - Position: %s, Orientation: %s" % (pprint.pformat(pose3.position),
    pprint.pformat(pose3.orientation)))