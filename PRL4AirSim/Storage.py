import msgpackrpc #install as admin: pip install msgpack-rpc-python
#import distributed.model.DQNTrainer as DQNTrainer
#https://linuxtut.com/en/70b626ca3ac6fbcdf939/
import numpy as np
import torch
import pathlib
import wandb
import DQNTrainer as DQNTrainer
import datetime
import time
import Utils as Utils
from collections import deque
import ReplayMemory as ReplayMemory

class Storage(object):
    def __init__(self):

        wandb.login()
        self.run_name = datetime.datetime.now().strftime("%Y_%m_%d_%Hh%Mm%Ss")
        self.run = wandb.init(
            project="2D-Drone-Multiple",
            config=Utils.getConfig(),
            name=self.run_name,
        )

        self.total_episodes = 0
        self.start_time = None
        self.agent = DQNTrainer.DQNTrainer(image_input_dims=Utils.getConfig()['state_space'],
                                           n_actions=Utils.getConfig()['action_space'],
                                           replayMemory_size=Utils.getConfig()['buffer_Size'],
                                           batch_size=Utils.getConfig()['batch_size'],
                                           learningRate=Utils.getConfig()['learning_rate'],
                                           discount_factor=Utils.getConfig()['discount_factor'],
                                           epsilon=1.0,
                                           replace_target_count_episode=Utils.getConfig()['replace_target_count_episode'])

        self.start_time = time.perf_counter()

    def pushMemory(self, state, action, next_state, reward, not_done):
        self.agent.memory.push(Utils.convertStateDicToNumpyDic(state), action, Utils.convertStateDicToNumpyDic(next_state), reward, not_done)
        if (len(self.agent.memory) % 100 == 0):
            wandb.log({"metric/Observations" : self.agent.memory.pushCounter},
                      step=self.total_episodes)

            if not len(self.agent.memory) == self.agent.memory.maxSize:
                print(len(self.agent.memory))

    def getMemoryPushCounter(self):
        return self.agent.memory.pushCounter

    def startSimulation(self):
        self.start_time = (time.perf_counter() / 3600)
        wandb.log({"metric/HoursRun" : 0,
                   "metric/Observations" : self.agent.memory.pushCounter},
                  step=self.total_episodes)
        print("============ START SIMULATION ===========")

    def getEpsilon(self):
        return self.agent.epsilon

    def finishEpisode(self, finalDistance, totalReward):
        self.total_episodes += 1
        self.agent.decrement_epsilon()
        wandb.log({
            "metric/Distance From Goal": finalDistance,
            "metric/Total Reward" : totalReward,
            "metric/Wall-Time /h" : (time.perf_counter()-self.start_time) / 3600.0,
            "metric/Epsilon" : self.agent.epsilon
        }, step=self.total_episodes)

        if self.total_episodes % 1000 == 0 and self.total_episodes != 0:
            print("saving model parameters in wandb")
            artifact = wandb.Artifact('dqn_3D_{}_EP_{}'.format(self.run_name, self.total_episodes), type='model', description='Episode {}'.format(self.total_episodes))
            artifact.add_file('{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve()))
            self.run.log_artifact(artifact)

    def setNetworkTrainIteration(self, trainIteration):
        wandb.log({
            "metric/Train Iteration": trainIteration
        }, step=self.total_episodes)

    def sampleFromStorage(self):
        if len(self.agent.memory) >= self.agent.replayMemory_size or len(self.agent.memory) >= self.agent.batch_size:
            sample = self.agent.memory.sample(self.agent.batch_size)
            batch = ReplayMemory.Transition(*zip(*sample))


            state = [Utils.convertStateDicToListDic(i) for i in batch.state]
            action = [int(i) for i in batch.action]
            next_state = [Utils.convertStateDicToListDic(i) for i in batch.next_state]
            reward = [float(i) for i in batch.reward]
            not_done = [int(i) for i in batch.not_done]

            return state, \
                   action, \
                   next_state, \
                   reward, \
                   not_done
        else:
            return None, None, None, None, None

    def confirmConnection(self):
        return 'Storage Server Connected!'

def testSampleFromStorage():
    storage_server = Storage()
    for i in range(50):
        storage_server.agent.memory.push({'image': np.zeros(shape=(32, 32)),
                                          'position': np.zeros(shape=(3,))},
                                         1,
                                         {'image': np.zeros(shape=(32, 32)),
                                          'position': np.zeros(shape=(3,))},
                                         0.1,
                                         1)
    storage_server.sampleFromStorage()

import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Storage",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("storage_port")
    args = parser.parse_args()
    arguments = vars(args)

    storage_server = Storage()
    server = msgpackrpc.Server(storage_server)
    server.listen(msgpackrpc.Address("127.0.0.1", int(arguments["storage_port"])))
    print("========== STARTING STORAGE SERVER ============")
    server.start()
    print("========== FINISH STORAGE SERVER ============")
    storage_server.run.finish()
