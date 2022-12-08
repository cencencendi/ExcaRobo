import gym
import collections
import math
import numpy as np
import os

from exca_envVelocity import ExcaBot
from stable_baselines3 import PPO

SIM_ON = 0

if __name__ == "__main__":
    env = ExcaBot(SIM_ON)

    log_path = os.path.join('Training', 'Logs')
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
    model.learn(total_timesteps=1000000)

    model_save_path = os.path.join('Training', 'Saved Models', 'PPO_25000(31)')
    model.save(model_save_path)