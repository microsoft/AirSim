import numpy as np
import airsim

import gym
from gym import spaces
from gym.utils import seeding


class AirSimEnv(gym.Env):
    metadata = {"render.modes": ["rgb_array"]}

    def __init__(self, image_shape):
        self.observation_space = spaces.Box(0, 255, shape=image_shape, dtype=np.uint8)
        self._seed()

        self.viewer = None
        self.steps = 0
        self.no_episode = 0
        self.reward_sum = 0

    def __del__(self):
        raise NotImplementedError()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _compute_reward(self):
        raise NotImplementedError()

    def step(self, action):
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def close(self):
        raise NotImplementedError()

    def render(self):
        return self._get_obs()
