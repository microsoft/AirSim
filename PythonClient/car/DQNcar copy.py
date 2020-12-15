import setup_path
import airsim
import numpy as np
import math
import time
from argparse import ArgumentParser

from airgym.envs.airsim_env import AirSimCarEnv


class AirSimDriveEnv(AirSimCarEnv):
    def __init__(self, ip_address, step_length, image_shape):
        super().__init__(self, ip_address, step_length, image_shape)

    def take_action(self, action):
        self.car_controls.brake = 0
        self.car_controls.throttle = 1
        if action == 0:
            self.car_controls.throttle = 0
            self.car_controls.brake = 1
        elif action == 1:
            self.car_controls.steering = 0
        elif action == 2:
            self.car_controls.steering = 0.5
        elif action == 3:
            self.car_controls.steering = -0.5
        elif action == 4:
            self.car_controls.steering = 0.25
        else:
            self.car_controls.steering = -0.25

        self.car.setCarControls(self.car_controls)

    def compute_reward(self):
        MAX_SPEED = 300
        MIN_SPEED = 10
        thresh_dist = 3.5
        beta = 3

        z = 0
        pts = [
            np.array([0, -1, z]),
            np.array([130, -1, z]),
            np.array([130, 125, z]),
            np.array([0, 125, z]),
            np.array([0, -1, z]),
            np.array([130, -1, z]),
            np.array([130, -128, z]),
            np.array([0, -128, z]),
            np.array([0, -1, z]),
        ]
        pd = car_state.kinematics_estimated.position
        car_pt = np.array([pd.x_val, pd.y_val, pd.z_val])

        dist = 10000000
        for i in range(0, len(pts) - 1):
            dist = min(
                dist,
                np.linalg.norm(np.cross((car_pt - pts[i]), (car_pt - pts[i + 1])))
                / np.linalg.norm(pts[i] - pts[i + 1]),
            )

        # print(dist)
        if dist > thresh_dist:
            reward = -3
        else:
            reward_dist = math.exp(-beta * dist) - 0.5
            reward_speed = (
                (car_state.speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED)
            ) - 0.5
            reward = reward_dist + reward_speed

        done = 0
        if reward < -1:
            done = 1
        if self.car_controls.brake == 0:
            if self.car_state.speed <= 5:
                done = 1

        return reward, done
