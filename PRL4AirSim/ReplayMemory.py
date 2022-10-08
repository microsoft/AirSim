import random
from collections import namedtuple, deque

#state_image, state_velocity, action, next_state_image, next_state_velocity, reward, not_done
Transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward', 'not_done'))

class ReplayMemory(object):
    def __init__(self, maxSize : int):
        self.maxSize = maxSize
        self.pushCounter = 0
        self.memory = deque([], maxlen=self.maxSize)

    def push(self, *args):
        """Save transition"""
        self.memory.append(Transition(*args))
        self.pushCounter += 1

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)