import os

from exca_SimpleEnv import ExcaBot
from stable_baselines3 import PPO

SIM_ON = 0

if __name__ == "__main__":
    env = ExcaBot(SIM_ON)

    log_path = os.path.join('Training', 'Logs', 'SimpleEnv')
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
    model.learn(total_timesteps=1500000)

    model_save_path = os.path.join('Training', 'Saved Models', 'PPO_Simple(11)')
    model.save(model_save_path)