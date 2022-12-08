import os

from exca_SimpleEnv import ExcaBot
from stable_baselines3 import A2C

SIM_ON = 0

if __name__ == "__main__":
    env = ExcaBot(SIM_ON)

    log_path = os.path.join('Training', 'Logs', 'SimpleEnv')
    model = A2C('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
    model.learn(total_timesteps=1000000)

    model_save_path = os.path.join('Training', 'Saved Models', 'A2C_Simple(1)')
    model.save(model_save_path)