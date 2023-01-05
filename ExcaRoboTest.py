# from ExcaRobo_4 import ExcaRobo
from ExcaRobo_3 import ExcaRobo
# from ExcaRobo_2 import ExcaRobo
# from ExcaRobo_1 import ExcaRobo
# from general_environment import ExcaRobo
from stable_baselines3 import PPO
import pybullet as p
import numpy as np

SIM_ON = 1

if __name__ == "__main__":
    env = ExcaRobo(SIM_ON)
    model = PPO.load('Training/Saved Models/InvKin(11)_3Joint3_Orientasi', env=env)
    obs = env.reset()
    score = 0
    done = False
    step = 0
    pose_all = []
    while not done:
        env.render()
        action, _ = model.predict(obs)
        obs, reward, done, info = env.step(action)
        score += reward
        step+=1
    orientation = env.normalize(-sum(env._get_joint_state()))
    print(f"Score: {score}, with step: {step}, pose: {np.array(p.getLinkState(env.boxId,4, computeLinkVelocity=1, computeForwardKinematics=1)[0])}, orientation: {orientation}")
    # p.disconnect()