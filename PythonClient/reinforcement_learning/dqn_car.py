import setup_path
import gym
import airgym

import numpy as np
import matplotlib.pyplot as plt
import time
import torch

from stable_baselines3 import DQN, PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

env = DummyVecEnv(
    [
        lambda: Monitor(
            gym.make(
                "airgym:airsim-car-sample-v0",
                ip_address="127.0.0.1",
                image_shape=(84, 84, 1),
            )
        )
    ]
)

model = DQN(
    "CnnPolicy",
    env,
    learning_rate=0.00025,
    verbose=1,
    batch_size=32,
    train_freq=4,
    target_update_interval=10000,
    learning_starts=200000,
    buffer_size=500000,
    max_grad_norm=10,
    exploration_fraction=0.1,
    exploration_final_eps=0.01,
    device="cuda",
    tensorboard_log="./tb_logs/",
)


callbacks = []
eval_callback = EvalCallback(
    env,
    callback_on_new_best=None,
    n_eval_episodes=5,
    best_model_save_path=".",
    log_path=".",
    eval_freq=10000,
)
callbacks.append(eval_callback)

kwargs = {}
kwargs["callback"] = callbacks

model.learn(total_timesteps=5e5, tb_log_name="dqn_run_" + str(time.time()), **kwargs)
model.save("dqn_airsim_car")
