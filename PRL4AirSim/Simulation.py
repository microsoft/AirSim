import Utils as Utils
import airsim
import numpy as np
import time
import DroneObj as DroneObj
import random
import argparse
from os.path import exists
import os
import pathlib

beforeTime = None
afterTime = None

class Sim(object):
    def __init__(self, image_shape, num_drones):
        self.image_shape = image_shape

        self.origin_UE = np.array([0.0, 0.0, 910.0])

        self.areans_train_long = np.array([
            # Using larger environment
            #[Utils.convert_pos_UE_to_AS(self.origin_UE, np.array([41156.0, 20459.0, 1000.0])), Utils.convert_pos_UE_to_AS(self.origin_UE, np.array([56206.0, 21019.0, 1000.0]))]
            # Using smaller environment
            [Utils.convert_pos_UE_to_AS(self.origin_UE, np.array([9030.0, -6760.0, 1000.0])), Utils.convert_pos_UE_to_AS(self.origin_UE, np.array([14060.0, -6760.0, 1000.0]))]
        ])

        self.areans = self.areans_train_long

        self.droneObjects = [DroneObj.DroneObject(i) for i in range(num_drones)]
        self.episodes = 0
        self.model_download_at_episode = 0
        self.numImagesSent = 0

        #TODO: HyperParameters
        self.step_length = 0.25
        self.constant_x_vel = 1.0
        self.constant_z_pos = Utils.convert_pos_UE_to_AS(origin_UE=self.origin_UE, pos_UE=[8600.0, -4160.0, 1510.0])[2]
        self.actionTime = 1.0
        self.resetBatch()


    def gatherAllObservations(self):
        useNewMethod = True

        nonResetingDrones = []
        for droneObject in self.droneObjects:
            if droneObject.reseting == False:
                nonResetingDrones.append(droneObject)

        if len(nonResetingDrones) == 0:
            return


        if useNewMethod:
            requests = [airsim.ImageRequest('depth_cam_{}'.format(droneObject.droneId), airsim.ImageType.DepthPlanar, True, True) for droneObject in nonResetingDrones]
            names = [droneObject.droneName for droneObject in nonResetingDrones]
            beforeTime = time.perf_counter()
            responses_raw = Utils.getClient().client.call('simGetBatchImages', requests, names)
            afterTime = time.perf_counter()

            print("Gather images: ", afterTime - beforeTime)
            responses = [airsim.ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]
            imageDepths = [airsim.list_to_2d_float_array(responses[i].image_data_float, responses[i].width, responses[i].height) for i in range(len(responses))]

        else:
            beforeTime = time.perf_counter()
            responses_raw = [Utils.getClient().client.call('simGetImages',
                                                           [airsim.ImageRequest('depth_cam_{}'.format(droneObject.droneId), airsim.ImageType.DepthPlanar, True, True)],
                                                           'Drone{}'.format(droneObject.droneId),
                                                           False) for droneObject in nonResetingDrones]
            afterTime = time.perf_counter()

            print("Gather images (old method): ", afterTime - beforeTime)
            responses = [airsim.ImageResponse.from_msgpack(response_raw[0]) for response_raw in responses_raw]
            imageDepths = [airsim.list_to_2d_float_array(responses[i].image_data_float, responses[i].width, responses[i].height) for i in range(len(responses))]

        for i, droneObject in enumerate(nonResetingDrones):
            imageDepth = imageDepths[i]
            if (imageDepth.size == 0):
                print("Image size is 0")
                imageDepth = np.ones(shape=(self.image_shape[1], self.image_shape[2])) * 30

            maxDistance = 50
            imageDepth[imageDepth > maxDistance] = maxDistance
            imageDepth = imageDepth.astype(np.uint8)

            if droneObject.currentStep == 0:
                droneObject.previous_depth_image = imageDepth

            stacked_images = np.array([imageDepth, droneObject.previous_depth_image])

            multirotorState = Utils.getClient().getMultirotorState(droneObject.droneName)
            velocity = multirotorState.kinematics_estimated.linear_velocity.to_numpy_array()
            droneObject.previous_depth_image = imageDepth

            droneObject.previousState = droneObject.currentState
            droneObject.currentState = {'image': stacked_images, 'velocity': velocity}
            droneObject.currentStatePos = multirotorState.kinematics_estimated.position.to_numpy_array()

    def doActionBatch(self):
        droneNames = []
        vx_vec = []
        vy_vec = []
        z_vec = []

        for droneObject in self.droneObjects:
                droneNames.append(droneObject.droneName)
                quad_vel = Utils.getClient().getMultirotorState(droneObject.droneName).kinematics_estimated.linear_velocity
                y_val_offset = 0
                if droneObject.currentAction == 0:
                    y_val_offset = self.step_length
                elif droneObject.currentAction == 1:
                    y_val_offset = -self.step_length


                vx_vec.append(self.constant_x_vel if droneObject.reseting == False else 0)
                vy_vec.append(quad_vel.y_val + y_val_offset if droneObject.reseting == False else 0)
                z_vec.append(self.constant_z_pos)
                droneObject.currentStep += 1


        Utils.getClient().simPause(False)
        Utils.getClient().client.call_async('moveByVelocityZBatch', vx_vec, vy_vec, z_vec, self.actionTime, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(), droneNames).join()
        Utils.getClient().simPause(True)

    def randomPoseInArena(self):
        width = 1600 // 100
        min = -(width // 2)
        max = (width // 2)
        return random.uniform(min, max)

    def resetBatch(self):
        windows = False

        # Size difference: -7710.0, -6070.0
        Utils.getClient().simPause(False)
        Utils.getClient().reset()
        time.sleep(5) if windows else time.sleep(0.25)

        randomArenas = np.random.randint(len(self.areans), size=len(self.droneObjects))

        for i in range(len(self.droneObjects)):
            self.droneObjects[i].currentArena = randomArenas[i]

        # airsim.Quaternionr(0.0, 0.0, 1.0, 0.0) = 180 degrees
        poses = [airsim.Pose(airsim.Vector3r(self.areans[droneObject.currentArena][0][0],
                                             self.areans[droneObject.currentArena][0][1] + self.randomPoseInArena(),
                                             self.areans[droneObject.currentArena][0][2]),
                             airsim.Quaternionr(0.0, 0.0, 0.0, 0.0)) for droneObject in self.droneObjects]

        Utils.getClient().client.call('simSetVehiclePoseBatch', poses, [droneObject.droneName for droneObject in self.droneObjects])

        time.sleep(5) if windows else time.sleep(0.25)

        for droneObject in self.droneObjects:
            Utils.getClient().armDisarm(True, droneObject.droneName)
            Utils.getClient().enableApiControl(True, droneObject.droneName)
            Utils.getClient().takeoffAsync(vehicle_name=droneObject.droneName)
            if windows: time.sleep(1)

            # Move up 3m
        time.sleep(5) if windows else time.sleep(0.25)

        for droneObject in self.droneObjects:
            quad_position = Utils.getClient().getMultirotorState(droneObject.droneName).kinematics_estimated.position
            #Utils.getClient().takeoffAsync(vehicle_name=droneObject.droneName).join()
            #Utils.getClient().hoverAsync(vehicle_name=droneObject.droneName).join()
            Utils.getClient().moveToPositionAsync(quad_position.x_val, quad_position.y_val, self.constant_z_pos, 3.0, vehicle_name=droneObject.droneName)
            droneObject.currentStep = 0
            currentPos_x_AS = Utils.getClient().getMultirotorState(droneObject.droneName).kinematics_estimated.position.to_numpy_array()[0]
            droneObject.distanceFromGoal = abs(currentPos_x_AS - self.areans[droneObject.currentArena][1][0])
            droneObject.reseting = False
            droneObject.currentTotalReward = 0
            if windows: time.sleep(1)

        #time.sleep(5)
        self.gatherAllObservations()
        time.sleep(5) if windows else time.sleep(0.25)

        Utils.getClient().simPause(True)
        self.episodes += 1

    def calculateReward(self, droneObject : DroneObj):
        image = droneObject.currentState['image']

        currentPos_x_AS = Utils.getClient().getMultirotorState(droneObject.droneName).kinematics_estimated.position.to_numpy_array()[0]
        distanceFromGoal = abs(currentPos_x_AS - self.areans[droneObject.currentArena][1][0])
        collisionInfo = Utils.getClient().simGetCollisionInfo(droneObject.droneName)

        hasCollided = collisionInfo.has_collided or image.min() < 0.55
        if droneObject.currentStep < 2:
            hasCollided = False

        done = 0
        reward_States = {
            "Collided": 0,
            "Won": 0,
            "approaching_collision": 0,
            "constant_reward" : 0,
            "max_actions" : 0,
            "goal_distance" : 0,
        }

        reward_States["goal_distance"] = 3.0

        if hasCollided:
            done = 1
            reward_States["Collided"] = -100
        elif distanceFromGoal <= 5:
            done = 1
            #reward_States["Won"] = 100
        elif droneObject.currentStep > 400:
            done = 1
            reward_States["max_actions"] = -10

        reward = sum(reward_States.values())
        droneObject.distanceFromGoal = distanceFromGoal
        droneObject.currentTotalReward += reward
        return reward, done

    def resetStep(self, droneObject : DroneObj):
        if droneObject.reseting == True:
            if droneObject.resetTick == 0 and time.perf_counter() - droneObject.resetingTime > 1:
                print("RESETING DRONE ", droneObject.droneId, print("len "), len(self.droneObjects))
                randomArena = np.random.randint(len(self.areans), size=(1,))[0]
                droneObject.currentArena = randomArena
                Utils.getClient().client.call_async("resetVehicle", droneObject.droneName, airsim.Pose(airsim.Vector3r(self.areans[droneObject.currentArena][0][0],
                                                                                                                 self.areans[droneObject.currentArena][0][1] + self.randomPoseInArena(),
                                                                                                                 self.areans[droneObject.currentArena][0][2]),
                                                                                                 airsim.Quaternionr(0.0, 0.0, 0.0, 0.0)))
                droneObject.resetTick = 1
                droneObject.resetingTime = time.perf_counter()
            if droneObject.resetTick == 1 and time.perf_counter() - droneObject.resetingTime > 1:
                Utils.getClient().armDisarm(True, droneObject.droneName)
                Utils.getClient().enableApiControl(True, droneObject.droneName)
                Utils.getClient().takeoffAsync(vehicle_name=droneObject.droneName)

                droneObject.resetingTime = droneObject.resetingTime
                droneObject.resetTick = 3


            if droneObject.resetTick == 3 and time.perf_counter() - droneObject.resetingTime > 2:
                droneObject.reseting = False
                droneObject.resetTick = 0

                state = Utils.getClient().getMultirotorState(droneObject.droneName)
                quad_position = state.kinematics_estimated.position
                Utils.getClient().moveToPositionAsync(quad_position.x_val, quad_position.y_val, self.constant_z_pos, 3.0, vehicle_name=droneObject.droneName)
                currentPos_x_AS = state.kinematics_estimated.position.to_numpy_array()[0]
                droneObject.distanceFromGoal = abs(currentPos_x_AS - self.areans[droneObject.currentArena][1][0])

                droneObject.currentStep = 0
                droneObject.currentTotalReward = 0
                self.episodes += 1



    def tick(self, agent):

        for droneObject in self.droneObjects:
            if droneObject.currentStatePos[0] < 5:
                droneObject.reseting = True

            self.resetStep(droneObject)

            if droneObject.reseting == False:
                maxAction, _ = agent.choose_action(droneObject.currentState)
                droneObject.currentAction = maxAction

        self.doActionBatch()
        self.gatherAllObservations()

        loadDQNFile = False

        for droneObject in self.droneObjects:
            if droneObject.reseting == False:
                self.numImagesSent += 1
                reward, done = self.calculateReward(droneObject)
                Utils.getModelServer().call_async("pushMemory",
                                                  Utils.convertStateDicToListDic(droneObject.previousState),
                                                  int(droneObject.currentAction),  #was considered np.int rather than int.
                                                  Utils.convertStateDicToListDic(droneObject.currentState),
                                                  reward,
                                                  1 - int(done))

                if done:
                    Utils.getModelServer().call_async("finishEpisode", droneObject.distanceFromGoal, droneObject.currentTotalReward)
                    droneObject.reseting = True
                    droneObject.resetingTime = time.perf_counter()
                    agent.epsilon = Utils.getModelServer().call("getEpsilon")
                    agent.memory.pushCounter = Utils.getModelServer().call("getMemoryPushCounter")
                    loadDQNFile = True

        if loadDQNFile and exists('{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve())):
            try:
                os.rename('{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve()), '{}/ModelSaves/dqn_read.pth'.format(pathlib.Path().resolve()))
                agent.load('{}/ModelSaves/dqn_read.pth'.format(pathlib.Path().resolve()))
                os.rename('{}/ModelSaves/dqn_read.pth'.format(pathlib.Path().resolve()), '{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve()))
            except:
                print("issue reading file")

        print("NumImagesSent: ", self.numImagesSent)




        finished = True
        for droneObject in self.droneObjects:
            if droneObject.reseting == False:
                finished = False

        finished = False

        return finished

#libUE4Editor-AirSim.so!_ZNSt3__110__function6__funcIZN3rpc6detail10dispatcher4bindIZN3msr6airlib22MultirotorRpcLibServerC1EPNS7_11ApiProviderENS_12basic_stringIcNS_11char_traitsIcEENS_9allocatorIcEEEEtE4$_14EEvRKSG_T_RKNS3_4tags14nonvoid_resultERKNSL_11nonzero_argEEUlRKN14clmdep_msgpack2v26objectEE_NSE_ISX_EEFNS_10unique_ptrINSS_2v113object_handleENS_14default_deleteIS11_EEEESW_EEclESW_()