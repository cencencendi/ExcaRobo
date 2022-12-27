import os

from ExcaRobo_4 import ExcaRobo
from stable_baselines3 import PPO

SIM_ON = 0

if __name__ == "__main__":
    env = ExcaRobo(SIM_ON)

    log_path = os.path.join('Training', 'Logs', 'Inverse_Kinematics')
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=log_path)
    model.learn(total_timesteps=10_000_000)

    model_save_path = os.path.join('Training', 'Saved Models', 'InvKin(11)_3Joint')
    model.save(model_save_path)
    print("Kelar brou")