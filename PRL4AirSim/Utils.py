import airsim
import numpy as np
import cv2 as cv
import msgpackrpc
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

# API call in AirSim can sometimes be broken depending on version, easier to call using RPC directly
def fixed_simGetImages(requests, vehicle_name = '', external : bool = False):
        responses_raw = getClient().client.call('simGetImages', requests, vehicle_name, external)
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
