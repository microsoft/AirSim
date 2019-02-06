import numpy
import cv2
import time
import sys
import os
import random
import glob
from airsim import *

def get_image(x, y, z, pitch, roll, yaw, client):
    """
    title::
        get_image

    description::
        Capture images (as numpy arrays) from a certain position.

    inputs::
        x
            x position in meters
        y
            y position in meters
        z
            altitude in meters; remember NED, so should be negative to be 
            above ground
        pitch
            angle (in radians); in computer vision mode, this is camera angle
        roll
            angle (in radians)
        yaw
            angle (in radians)
        client
            connection to AirSim (e.g., client = MultirotorClient() for UAV)

    returns::
        position
            AirSim position vector (access values with x_val, y_val, z_val)
        angle
            AirSim quaternion ("angles")
        im
            segmentation or IR image, depending upon palette in use (3 bands)
        imScene
            scene image (3 bands)

    author::
        Elizabeth Bondi
        Shital Shah
    """

    #Set pose and sleep after to ensure the pose sticks before capturing image.
    client.simSetVehiclePose(Pose(Vector3r(x, y, z), \
                      to_quaternion(pitch, roll, yaw)), True)
    time.sleep(0.1)

    #Capture segmentation (IR) and scene images.
    responses = \
        client.simGetImages([ImageRequest("0", ImageType.Infrared,
                                          False, False),
                            ImageRequest("0", ImageType.Scene, \
                                          False, False),
                            ImageRequest("0", ImageType.Segmentation, \
                                          False, False)])

    #Change images into numpy arrays.
    img1d = numpy.fromstring(responses[0].image_data_uint8, dtype=numpy.uint8)
    im = img1d.reshape(responses[0].height, responses[0].width, 4) 

    img1dscene = numpy.fromstring(responses[1].image_data_uint8, dtype=numpy.uint8)
    imScene = img1dscene.reshape(responses[1].height, responses[1].width, 4)

    return Vector3r(x, y, z), to_quaternion(pitch, roll, yaw),\
           im[:,:,:3], imScene[:,:,:3] #get rid of alpha channel

def main(client,
         objectList,
         pitch=numpy.radians(270), #image straight down
         roll=0,
         yaw=0,
         z=-122,
         writeIR=False,
         writeScene=False,
         irFolder='',
         sceneFolder=''):
    """
    title::
        main

    description::
        Follow objects of interest and record images while following.

    inputs::
        client
            connection to AirSim (e.g., client = MultirotorClient() for UAV)
        objectList
            list of tag names within the AirSim environment, corresponding to 
            objects to follow (add tags by clicking on object, going to 
            Details, Actor, and Tags, then add component)
        pitch
            angle (in radians); in computer vision mode, this is camera angle
        roll
            angle (in radians)
        yaw
            angle (in radians)
        z
            altitude in meters; remember NED, so should be negative to be 
            above ground
        write
            if True, will write out the images
        folder
            path to a particular folder that should be used (then within that
            folder, expected folders are ir and scene)

    author::
        Elizabeth Bondi
    """
    i = 0
    for o in objectList:
        startTime = time.time()
        currentTime = time.time() - startTime
        pose = client.simGetObjectPose(o);

        #Capture images for a certain amount of time in seconds (half hour now)
        while currentTime < 1800:
            #Capture image - pose.position x_val access may change w/ AirSim
            #version (pose.position.x_val new, pose.position[b'x_val'] old)
            vector, angle, ir, scene = get_image(pose.position.x_val, 
                                                 pose.position.y_val, 
                                                 z, 
                                                 pitch, 
                                                 roll, 
                                                 yaw, 
                                                 client)

            #Convert color scene image to BGR for write out with cv2.
            r,g,b = cv2.split(scene)
            scene = cv2.merge((b,g,r))

            if writeIR:
                cv2.imwrite(irFolder+'ir_'+str(i).zfill(5)+'.png', ir)
            if writeScene:
                cv2.imwrite(sceneFolder+'scene_'+str(i).zfill(5)+'.png', 
                            scene)

            i += 1
            currentTime = time.time() - startTime
            pose = client.simGetObjectPose(o);



if __name__ == '__main__':
    
    #Connect to AirSim, UAV mode.
    client = MultirotorClient()
    client.confirmConnection()

    #Tags for poachers in each of the three groups in Africa enviornment.
    objectList = ['Poacher1A', 'Poacher1B', 'Poacher1C']

    #Sample calls to main, varying camera angle and altitude.
    #straight down, 400ft
    main(client, 
         objectList, 
         folder=r'auto\winter\400ft\down') 
    #straight down, 200ft
    main(client, 
         objectList, 
         z=-61, 
         folder=r'auto\winter\200ft\down') 
    #45 degrees, 200ft -- note that often object won't be scene since position
    #is set exactly to object's
    main(client, 
         objectList, 
         z=-61, 
         pitch=numpy.radians(315), 
         folder=r'auto\winter\200ft\45') 
    #45 degrees, 400ft -- note that often object won't be scene since position
    #is set exactly to object's
    main(client, 
         objectList, 
         pitch=numpy.radians(315), 
         folder=r'auto\winter\400ft\45') 