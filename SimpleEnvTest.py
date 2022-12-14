import gym
import collections
import math
import numpy as np
import os
import pybullet as p

from exca_SimpleEnv import ExcaBot
from stable_baselines3 import PPO

SIM_ON = 1

if __name__ == "__main__":
    env = ExcaBot(SIM_ON)
    model = PPO.load('Training/Saved Models/PPO_Simple(11)', env=env)
    obs = env.reset()
    score = 0
    done = False
    step = 0
    while not done:
        env.render()
        action, _ = model.predict(obs)
        obs, reward, done, info = env.step(action)
        score += reward
        step+=1
    print(f"Score: {score}, with step: {step}, error: {obs[1]}")
    # p.disconnect()