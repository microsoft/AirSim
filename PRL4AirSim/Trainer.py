import msgpackrpc #install as admin: pip install msgpack-rpc-python
#import distributed.model.DQNTrainer as DQNTrainer
#https://linuxtut.com/en/70b626ca3ac6fbcdf939/
import torch
import pathlib
import DQNTrainer as DQNTrainer
import datetime
import time
import Utils as Utils
from collections import deque
import ReplayMemory as ReplayMemory

import os
from os.path import exists

class Trainer(object):
    def __init__(self):
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

    def confirmConnection(self):
        return 'Model Server Connected!'

    def learn(self):
        return

    def saveModel(self):
        return

def testSampleFromStorageTrainer():
    import Storage
    import numpy as np
    storage_server = Storage.Storage()
    for i in range(50):
        storage_server.agent.memory.push({'image': np.zeros(shape=(2, 32, 32)),
                                          'velocity': np.zeros(shape=(3,))},
                                         1,
                                         {'image': np.zeros(shape=(2, 32, 32)),
                                          'velocity': np.zeros(shape=(3,))},
                                         0.1,
                                         1)
    state, action, next_state, reward, not_done = storage_server.sampleFromStorage()

    transitions = []
    for i in range(len(state)):
        transition = ReplayMemory.Transition(Utils.convertStateDicToNumpyDic(state[i]),
                                                     action[i],
                                                     Utils.convertStateDicToNumpyDic(next_state[i]),
                                                     reward[i],
                                                     not_done[i])
        transitions.append(transition)
    trainer = Trainer()
    trainer.agent.learn(transitions)


import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Storage",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("storage_port")
    args = parser.parse_args()
    arguments = vars(args)

    run_tests = False
    if run_tests:
        testSampleFromStorageTrainer()

    print("========== STARTING TRAINING CLIENT ============")
    trainer = Trainer()
    try:
        model_server = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", int(arguments["storage_port"])))
        print(model_server.call("confirmConnection"))
    except Exception as e:
        print("Cannot connect to the model server, please ")
        print("Ip address = {} and port {}".format("127.0.0.1", int(arguments["storage_port"])))
        print(e)
        exit(1)

    trainIteration = 0
    previous_time = time.perf_counter()
    while True:
        state, action, next_state, reward, not_done = model_server.call("sampleFromStorage")
        if state == None:
            print("Waiting for transitions")
            time.sleep(2)
        else:
            transitions = []
            for i in range(len(state)):
                transition = ReplayMemory.Transition(Utils.convertStateDicToNumpyDic(state[i]),
                                                     action[i],
                                                     Utils.convertStateDicToNumpyDic(next_state[i]),
                                                     reward[i],
                                                     not_done[i])
                transitions.append(transition)

            trainer.agent.learn(transitions)
            trainIteration += 1
            if trainIteration % 200 == 0:
                model_server.call("setNetworkTrainIteration", trainIteration)
                print("Saving model")
                #torch.save(trainer.agent.network.state_dict(), '{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve()))
                print("train iteration ", trainIteration, time.perf_counter() - previous_time)

                if exists('{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve())):
                    os.rename('{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve()), '{}/ModelSaves/dqn_read.pth'.format(pathlib.Path().resolve()))
                    torch.save(trainer.agent.network.state_dict(), '{}/ModelSaves/dqn_read.pth'.format(pathlib.Path().resolve()))
                    os.rename('{}/ModelSaves/dqn_read.pth'.format(pathlib.Path().resolve()), '{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve()))
                else:
                    torch.save(trainer.agent.network.state_dict(), '{}/ModelSaves/dqn.pth'.format(pathlib.Path().resolve()))

        previous_time = time.perf_counter()
