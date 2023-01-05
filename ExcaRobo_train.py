import os

from ExcaRobo_1 import ExcaRobo
from stable_baselines3 import PPO

SIM_ON = 0

if __name__ == "__main__":
    env = ExcaRobo(SIM_ON)

    log_path = os.path.join('Training', 'Logs', 'Inverse_Kinematics')
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
    model.learn(total_timesteps=5_000_000)

    model_save_path = os.path.join('Training', 'Saved Models', 'InvKin(13)_3Joint1_Orientasi')
    model.save(model_save_path)
    print("Kelar brou")