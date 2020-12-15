import setup_path
import airsim
import numpy as np
import math
import time
from argparse import ArgumentParser
from airgym.envs.airsim_env import AirSimEnv


class AirSimCarEnv(AirSimEnv):
    def __init__(self, ip_address, image_shape):
        super().__init__(image_shape)

        self.image_shape = image_shape
        self.start_ts = 0

        self.state = {
            "position": np.zeros(3),
            "collision": False,
            "prev_position": np.zeros(3),
        }

        self.car = airsim.CarClient(ip=ip_address)
        self.action_space = spaces.Discrete(6)

        self.image_request = airsim.ImageRequest(
            "front_center", airsim.ImageType.Scene, False, False
        )

        self.car_controls = airsim.CarControls()
        self.car_state = None

        self.state["pose"] = None
        self.state["prev_pose"] = None
        self.state["collision"] = None

    def __del__(self):
        self.car.reset()

    def _do_action(self, action):
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

    def _get_obs(self):
        response = self.drone.simGetImages([self.image_request])
        image = np.reshape(
            np.fromstring(response[0].image_data_uint8, dtype=np.uint8),
            self.image_shape,
        )
        self.car_state = self.car.getCarState()
        collision = self.car.simGetCollisionInfo().has_collided

        self.state["prev_pose"] = self.state["pose"]
        self.state["pose"] = self.car_state.kinematics_estimated
        self.state["collision"] = collision

        return image

    def _compute_reward(self):
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
        pd = self.state["pose"].position
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
                (self.car_state.speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED)
            ) - 0.5
            reward = reward_dist + reward_speed

        done = 0
        if reward < -1:
            done = 1
        if self.car_controls.brake == 0:
            if self.car_state.speed <= 5:
                done = 1

        return reward, done
