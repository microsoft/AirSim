import numpy as np
import airsim

import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym.spaces import Tuple, Box, Discrete, MultiDiscrete

from collections import OrderedDict


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


class AirSimDroneEnv(AirSimEnv):
    def __init__(
        self, ip_address, action_type, control_type, step_length, image_shape, goal
    ):
        super().__init__(image_shape)

        self.step_length = step_length
        self.action_type = action_type
        self.control_type = control_type
        self.image_shape = image_shape
        self.goal = airsim.Vector3r(goal[0], goal[1], goal[2])
        self.start_ts = 0

        if self.action_type is "discrete":
            self.action_space = spaces.Discrete(6)

            if control_type is not "position" and not "velocity":
                print(
                    "Invalid control type for discrete actions. Defaulting to position"
                )
                self.control_type = "position"
            else:
                self.control_type = control_type

        elif self.action_type is "continuous":
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,))
            if control_type is not "rate" and not "pwm":
                print("Invalid control type for continuous actions. Defaulting to rate")
                self.control_type = "rate"
            else:
                self.control_type = control_type
        else:
            print(
                "Must choose an action type {'discrete','continuous'}. Defaulting to discrete."
            )
            self.action_space = spaces.Discrete(6)
            self.control_type = "position"

        self.state = {
            "position": np.zeros(3),
            "collision": False,
            "prev_position": np.zeros(3),
        }

        self.drone = airsim.MultirotorClient(ip=ip_address)
        self._setup_flight()

        self.image_request = airsim.ImageRequest(
            "front_center", airsim.ImageType.Scene, False, False
        )

        # List of discrete actions
        self.forward = [self.step_length, 0, 0]
        self.backward = [-self.step_length, 0, 0]
        self.left = [0, -self.step_length, 0]
        self.right = [0, self.step_length, 0]
        self.up = [0, 0, -self.step_length]
        self.down = [0, 0, self.step_length]

        self.discrete_action_dict = {
            0: self.forward,
            1: self.backward,
            2: self.right,
            3: self.left,
            4: self.up,
            5: self.down,
        }

    def __del__(self):
        self.drone.reset()

    def _setup_flight(self):
        self.drone.reset()
        self.drone.enableApiControl(True)
        self.drone.armDisarm(True)
        self.drone.moveToPositionAsync(0, 0, -2, 2).join()

    def _get_obs(self):
        response = self.drone.simGetImages([self.image_request])
        image = np.reshape(
            np.fromstring(response[0].image_data_uint8, dtype=np.uint8),
            self.image_shape,
        )
        _drone_state = self.drone.getMultirotorState()
        position = _drone_state.kinematics_estimated.position.to_numpy_array()
        collision = self.drone.simGetCollisionInfo().has_collided

        self.state["prev_position"] = self.state["position"]
        self.state["position"] = position
        self.state["collision"] = collision

        return image

    def _compute_reward(self):
        pos = self.state["position"]
        current_pos = airsim.Vector3r(pos[0], pos[1], pos[2])
        done = False

        if current_pos == self.goal:
            done = True
            reward = 100
            return reward, done
        elif self.state["collision"] == True:
            done = True
            reward = -100

        dist = current_pos.distance_to(self.goal)
        reward += abs(pos[1]) - abs(prev_pos[1])

        return reward, done

    def _do_action(self, action):
        if self.action_type is "discrete":
            command = self.action_to_command(action)
            if self.control_type is "position":
                pos = self.state["position"]
                self.pose.position.x_val = float(pos[0] + action[0])
                self.pose.position.y_val = float(pos[1] + action[1])
                self.pose.position.z_val = float(pos[2] + action[2])
                self.drone.simSetVehiclePose(self.pose, False)

            elif self.control_type is "velocity":
                self.drone.moveByVelocityZAsync(
                    float(action[0]),
                    float(action[1]),
                    float(action[2]),
                    self.step_length,
                ).join()

        elif self.action_type is "continuous":
            if self.control_type is "rate":
                self.drone.moveByRollPitchYawrateThrottleAsync(
                    float(action[0]),
                    float(action[1]),
                    float(action[2]),
                    float(action[3]),
                    self.step_length,
                ).join()

            elif self.control_type == "pwm":
                self.drone.moveByMotorPWMsAsync(
                    float(action[0]),
                    float(action[1]),
                    float(action[2]),
                    float(action[3]),
                )

    def step(self, action):
        self._do_action(action)
        obs = self._get_obs()
        reward, done = self._compute_reward()

        return obs, reward, done, self.state

    def reset(self):
        self._setup_flight()
        self._get_obs()

    def actions_to_op(self, action):
        return self.discrete_action_dict[action]
