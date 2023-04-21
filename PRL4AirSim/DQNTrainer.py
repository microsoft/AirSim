import numpy as np
import torch
#import distributed.distributed_22_06_02.ReplayMemory as ReplayMemory
#import distributed.distributed_22_06_02.model.DQNetwork as DQNetwork
import ReplayMemory as ReplayMemory
import DQNetwork as DQNetwork

class DQNTrainer(object):
    def __init__(self, image_input_dims : np.array,
                 n_actions : int,
                 replayMemory_size : int,
                 batch_size : int,
                 learningRate : float = 0.01,
                 discount_factor : float = 0.99,
                 epsilon : float = 1.0,
                 replace_target_count_episode : int = 100,
                 save_model_count_episode : int = 250,
                 checkpoint_episode : int = 250,
                 checkpoint_file : str = 'model_saves/dqn',
                 number_dimensions : int = 2):

        self.image_input_dims = image_input_dims
        self.n_actions = n_actions
        self.discount_factor = discount_factor
        self.epsilon = epsilon
        self.replace_target_count_episode = replace_target_count_episode
        self.save_model_count_episode = save_model_count_episode
        self.network = DQNetwork.DQNetwork(learningRate, self.n_actions, image_input_dims)
        self.target_network = DQNetwork.DQNetwork(learningRate, self.n_actions, image_input_dims)
        self.batch_size = batch_size
        self.memory = ReplayMemory.ReplayMemory(replayMemory_size)
        self.replayMemory_size = replayMemory_size
        self.checkpoint_episode = checkpoint_episode
        self.checkpoint_file = checkpoint_file

    def load(self, state_dict):
        self.network.load_state_dict(state_dict=torch.load(state_dict))
        self.target_network.load_state_dict(state_dict=torch.load(state_dict))
        print("Loaded from state dictionary")

    # Epsilon Greedy action selection.
    def choose_action(self, observation : dict):
        maxValue = None
        # Expecting (Batch, Channels, Height, Width)
        image = torch.tensor(np.reshape(np.array(observation['image']), (1, *self.image_input_dims)), dtype=torch.float).to(self.network.device)
        velocity = torch.tensor(np.array(observation['velocity']).reshape((1, 3)), dtype=torch.float).to(self.network.device)
        actions = self.network.forward(image, velocity)

        if np.random.random() > self.epsilon:
            action = torch.argmax(actions).item()
        else:
            action = np.random.choice([i for i in range(self.n_actions)])

        #action = torch.argmax(actions).item()

        maxValue = torch.max(actions).item()
        #self.decrement_epsilon()
        return action, maxValue

    def learn(self, transitions):

        self.network.optimizer.zero_grad()
        self.memory.pushCounter += 1

        if self.memory.pushCounter % self.replace_target_count_episode == 0:
            print("Transfer weights to target network at step {}".format(self.memory.pushCounter))
            self.target_network.load_state_dict(self.network.state_dict())

        batch = ReplayMemory.Transition(*zip(*transitions))

        state = (torch.tensor(np.array([i[b'image'].reshape(*self.image_input_dims) for i in batch.state])).to(self.network.device).float(),
                 torch.tensor(np.array([i[b'velocity'] for i in batch.state])).to(self.network.device).float())

        next_state = (torch.tensor(np.array([i[b'image'].reshape(*self.image_input_dims) for i in batch.next_state])).to(self.network.device).float(),
                      torch.tensor(np.array([i[b'velocity'] for i in batch.next_state])).to(self.network.device).float())

        actions = torch.tensor(batch.action).to(self.network.device)
        rewards = torch.tensor(batch.reward).to(self.network.device)
        not_done = torch.tensor(batch.not_done).to(self.network.device)

        indices = np.arange(self.batch_size)

        # https://en.wikipedia.org/wiki/Q-learning
        # Old quality value
        Q_old = self.network.forward(*state)[indices, actions]
        Q_target = rewards + self.target_network.forward(*next_state).max(dim=1)[0] * self.discount_factor * not_done

        loss = self.network.loss(Q_old.double(), Q_target.double()).to(self.network.device)
        loss.backward()
        self.network.optimizer.step()

    def decrement_epsilon(self):
        #if self.memory.pushCounter < self.replayMemory_size and self.memory.pushCounter > self.replayMemory_size * 0.2 * 0.99:
        if self.memory.pushCounter > self.replayMemory_size:
            self.epsilon = max(0, 1. - ((self.memory.pushCounter - self.replayMemory_size) / self.replayMemory_size))