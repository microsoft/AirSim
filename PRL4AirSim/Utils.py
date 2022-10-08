import airsim
import numpy as np
import cv2 as cv
import time
import datetime
import msgpackrpc
from DroneObj import DroneObject
import json

config = json.load(open("config.json", "r"))
print(config)
client = None
model_server = None

def connectClient(trainer_ip_address, ue_ip_address, trainer_port = 29000, ue_port = 41451):
    global client, model_server
    try:
        client = airsim.MultirotorClient(ip=ue_ip_address, port=ue_port)
        client.confirmConnection()
    except Exception as e:
        print("Cannot Connect to Multirotor Client, please ensure Unreal Engine is running with AirSim plugin")
        print("Ip address = {} and port {}".format(ue_ip_address, ue_port))
        print(e)
        exit(1)

    try:
        model_server = msgpackrpc.Client(msgpackrpc.Address(trainer_ip_address, trainer_port))
        print(model_server.call("confirmConnection"))
    except Exception as e:
        print("Cannot connect to the model server, please ")
        print("Ip address = {} and port {}".format(trainer_ip_address, trainer_port))
        print(e)
        exit(1)

    return client, model_server

def getClient() -> airsim.MultirotorClient:
    return client

def getModelServer() -> msgpackrpc.Client:
    return model_server

def getConfig():
    return config

def convertStateDicToListDic(state):
    listState = {}
    for key in state:
        listState[key] = state[key].tolist()
        #print(listState)
    return listState

def convertStateDicToNumpyDic(state):
    listState = {}
    for key in state:
        listState[key] = np.array(state[key])
        #print(listState)
    return listState

def teleportDrone(droneObject : DroneObject, client : airsim.MultirotorClient, position : np.array = np.array([0.0,0.0,0.0]), orientation : np.array = np.array([0.0,0.0,0.0,0.0]), delay = True):
    getClient().armDisarm(False, droneObject.droneName)
    getClient().enableApiControl(False, droneObject.droneName)
    client.simSetVehiclePose(pose=airsim.Pose(
                                 airsim.Vector3r(position[0] - droneObject.droneSpawnOffset[0], position[1] - droneObject.droneSpawnOffset[1], position[2] - droneObject.droneSpawnOffset[2]),
                                 airsim.Quaternionr(orientation[0], orientation[1], orientation[2], orientation[3])),
                            ignore_collision=True,
                            vehicle_name=droneObject.droneName)
    if delay:
        time.sleep(1.0)
    #time.sleep(1.0)
    client.enableApiControl(True, droneObject.droneName)
    client.armDisarm(True, droneObject.droneName)

# THE API CALL IS BROKEN
def fixed_simGetImages(requests, vehicle_name = '', external : bool = False):
        responses_raw = getClient().client.call('simGetImages', requests, vehicle_name, external)
        return [airsim.ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]

def simGetBatchImages(requests, vehicle_names):
        responses_raw = getClient().client.call('simGetBatchImages', requests, vehicle_names)
        return [airsim.ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]


def handleImage(droneName : str, cameraName : str, imageType : airsim.ImageType) -> np.array:
    if (imageType == airsim.ImageType.Scene):
        imageRequests = [airsim.ImageRequest(cameraName, imageType, False, False)]
        imageResponses = fixed_simGetImages(imageRequests, droneName, False)
        image1d = np.fromstring(imageResponses[0].image_data_uint8, dtype=np.uint8)
        imageRGB = image1d.reshape((imageResponses[0].height, imageResponses[0].width, 3))
        return imageRGB
    elif (imageType == airsim.ImageType.DepthPlanar or imageType == airsim.ImageType.DepthVis or imageType == airsim.ImageType.DepthPerspective):
        imageResponses = fixed_simGetImages([airsim.ImageRequest(cameraName, airsim.ImageType.DepthPlanar, True, True)], droneName, False)
        imageDepth = airsim.list_to_2d_float_array(imageResponses[0].image_data_float, imageResponses[0].width, imageResponses[0].height)
        return imageDepth
    else:
        print("NOT CODED THE HANDLING OF THIS IMAGE TYPE YET")
        return np.array([])

def handleImages(droneName : str, cameraNames : [], imageType : airsim.ImageType) -> np.array:
    if (imageType == airsim.ImageType.Scene):
        imageRequests = [airsim.ImageRequest(cameraName, imageType, False, False) for cameraName in cameraNames]
        imageResponses = fixed_simGetImages(imageRequests, droneName, False)
        image1d = np.fromstring(imageResponses[0].image_data_uint8, dtype=np.uint8)
        imageRGB = image1d.reshape((imageResponses[0].height, imageResponses[0].width, 3))
        return imageRGB
    elif (imageType == airsim.ImageType.DepthPlanar or imageType == airsim.ImageType.DepthVis or imageType == airsim.ImageType.DepthPerspective):
        imageResponses = fixed_simGetImages([airsim.ImageRequest(cameraName, airsim.ImageType.DepthPlanar, True, True) for cameraName in cameraNames], droneName, False)
        imageDepths = [airsim.list_to_2d_float_array(imageResponses[i].image_data_float, imageResponses[i].width, imageResponses[i].height) for i in range(len(cameraNames))]
        return imageDepths
    else:
        print("NOT CODED THE HANDLING OF THIS IMAGE TYPE YET")
        return np.array([])

def showRGBImage(droneName : str):
    image = handleImage(droneName, 'scene_cam', airsim.ImageType.Scene)
    cv.imshow("RGB image", image)
    cv.waitKey(0)

def showDepthImage(droneName : str):
    imageResponses = fixed_simGetImages([airsim.ImageRequest('depth_cam', airsim.ImageType.DepthPlanar, True, True)], droneName, False)
    imageDepth = airsim.list_to_2d_float_array(imageResponses[0].image_data_float, imageResponses[0].width,
                                               imageResponses[0].height)
    cv.imshow("depth image", imageDepth)
    cv.waitKey(0)

def convert_pos_UE_to_AS(origin_UE : np.array, pos_UE : np.array):
    pos = np.zeros(3, dtype=np.float)
    pos[0] = pos_UE[0] - origin_UE[0]
    pos[1] = pos_UE[1] - origin_UE[1]
    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100

def getCurrentTimeStr():
    return datetime.datetime.now().strftime("%H:%M:%S")

def showDifference():
    last_image = None
    start = True
    while True:
        imageDepth = handleImage(droneName='Drone0', cameraName='depth_cam', imageType=airsim.ImageType.DepthPlanar)
        #imageDepth = handleImages(droneName='Drone0', cameraNames=['depth_cam'], imageType=airsim.ImageType.DepthPlanar)

        maxDistance = 30
        imageDepth[imageDepth > maxDistance] = maxDistance

        if start == True:
            start = False
            last_image = imageDepth

        subtract = np.subtract(imageDepth, last_image)
        difference = np.add(subtract, np.ones(shape=(64, 64)) * 30)
        difference *= (255/60)
        difference = np.array(difference, dtype=np.uint8)

        difference = np.reshape(difference, (64, 64, 1))
        ## This is required to show an image (as using a float will cause it to normalise between the values of [0,1]
        ## https://docs.opencv.org/4.x/d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563


        last_image = imageDepth

        cv.imshow("depth image", difference)
        cv.waitKey(100)

def showImageBatch1():
    while True:
        beforeTime = time.perf_counter()
        #imageDepth = handleImage(droneName='Drone0', cameraName='depth_cam_0', imageType=airsim.ImageType.DepthPlanar)
        #imageDepth = handleImage(droneName='Drone1', cameraName='depth_cam_1', imageType=airsim.ImageType.DepthPlanar)
        #imageDepth = handleImage(droneName='Drone2', cameraName='depth_cam_2', imageType=airsim.ImageType.DepthPlanar)
        #imageDepth = handleImage(droneName='Drone3', cameraName='depth_cam_3', imageType=airsim.ImageType.DepthPlanar)
        num_images = 5
        for i in range(num_images):
            imageResponses = fixed_simGetImages([airsim.ImageRequest('depth_cam_{}'.format(i), airsim.ImageType.DepthPlanar, True, True)], 'Drone{}'.format(i), False)


        # imageDepth = handleImages(droneName='Drone0', cameraNames=['depth_cam'], imageType=airsim.ImageType.DepthPlanar)
        afterTime = time.perf_counter()
        print("Difference {}".format(afterTime - beforeTime))
        imageDepth = handleImage(droneName='Drone0', cameraName='depth_cam_0', imageType=airsim.ImageType.DepthPlanar)
        maxDistance = 50
        imageDepth[imageDepth > maxDistance] = maxDistance
        imageDepth *= (255/maxDistance)

        imageDepth = np.array(imageDepth, dtype=np.uint8)
        imageDepth = np.reshape(imageDepth, (64, 64, 1))
        ## This is required to show an image (as using a float will cause it to normalise between the values of [0,1]
        ## https://docs.opencv.org/4.x/d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563


        cv.imshow("depth image", imageDepth)
        cv.waitKey(1000)

def showImageBatch2():
    getClient().enableApiControl(True, vehicle_name='Drone0')
    getClient().armDisarm(True, vehicle_name='Drone0')
    getClient().takeoffAsync(vehicle_name='Drone0')
    while True:
        getClient().takeoffAsync(vehicle_name='Drone0')
        num_images = 5
        requests = [airsim.ImageRequest('depth_cam_{}'.format(i), airsim.ImageType.DepthPlanar, True, True) for i in range(num_images)]
        names = ['Drone{}'.format(i) for i in range(num_images)]

        beforeTime = time.perf_counter()
        responses_raw = getClient().client.call('simGetBatchImages', requests, names)
        responses = [airsim.ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]
        #print(responses)
        afterTime = time.perf_counter()
        print("Difference {}".format(afterTime - beforeTime))

        imageDepth = airsim.list_to_2d_float_array(responses[0].image_data_float, responses[0].width, responses[0].height)
        maxDistance = 50
        imageDepth[imageDepth > maxDistance] = maxDistance
        imageDepth *= (255/maxDistance)

        imageDepth = np.array(imageDepth, dtype=np.uint8)
        imageDepth = np.reshape(imageDepth, (64, 64, 1))
        ## This is required to show an image (as using a float will cause it to normalise between the values of [0,1]
        ## https://docs.opencv.org/4.x/d7/dfc/group__highgui.html#ga453d42fe4cb60e5723281a89973ee563


        cv.imshow("depth image", imageDepth)
        cv.waitKey(1000)


def showAllImagesBatch():
    while True:
        num_drones = 15
        requests = [airsim.ImageRequest('depth_cam_{}'.format(i), airsim.ImageType.DepthPlanar, True, True) for i in range(num_drones)]
        names = ['Drone{}'.format(i) for i in range(num_drones)]
        responses_raw = getClient().client.call('simGetBatchImages', requests, names)
        responses = [airsim.ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]
        imageDepths = []

        for response in responses:
            imageDepth = airsim.list_to_2d_float_array(response.image_data_float, response.width,
                                                       response.height)
            maxDistance = 50
            imageDepth[imageDepth > maxDistance] = maxDistance
            imageDepth *= (255 / maxDistance)

            imageDepth = np.array(imageDepth, dtype=np.uint8)
            imageDepth = np.reshape(imageDepth, (64, 64, 1))
            imageDepth = cv.resize(imageDepth, (128, 128), interpolation = cv.INTER_AREA)
            imageDepths.append(imageDepth)

        image = cv.hconcat([imageDepths[0], imageDepths[1], imageDepths[2]])


        columns = 5
        rows = 3

        images = []

        for row in range(rows):
            images.append(cv.hconcat([imageDepths[i * row] for i in range(columns)]))

        image = cv.vconcat([images[i] for i in range(len(images))])


        cv.imshow("All Drones", image)
        cv.waitKey(100)

