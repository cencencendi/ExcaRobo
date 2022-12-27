import gym
import math
import time
import numpy as np
import pybullet as p
from gym import spaces
import pybullet_data

class ExcaRobo(gym.Env):
    def __init__(self, sim_active):
        super(ExcaRobo, self).__init__()
        self.sim_active = sim_active
        if self.sim_active:
               physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        else:
            physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version

        self.MAX_EPISODE = 5_000
        self.dt = 1.0/240.0
        self.max_theta = [3.1, 1.03, 1.51]    
        self.min_theta = [-3.1, -0.954, -0.1214]
        self.observation_space = spaces.Box(low =-np.inf, high = np.inf, shape=(12,), dtype=np.float32)
        self.action_space = spaces.Box(low = -0.5, high = 0.5, shape=(2,), dtype=np.float32)
        self.steps_left = np.copy(self.MAX_EPISODE)
        self.state = np.zeros(5) #[theta1, theta2, x, y, z]
        self.position_target = np.array([6,0,7]) #theta0 = joint1, theta1 = joint2, theta2 = joint3, theta3 = joint4
        self.start_simulation()

    def step(self, action):
        # p.setJointMotorControl2(self.boxId, 1 , p.VELOCITY_CONTROL, targetVelocity = action[0], force= 50_000)
        p.setJointMotorControl2(self.boxId, 2 , p.VELOCITY_CONTROL, targetVelocity = action[0], force= 250_000)
        p.setJointMotorControl2(self.boxId, 3 , p.VELOCITY_CONTROL, targetVelocity = action[1], force= 250_000)
        # p.setJointMotorControl2(self.boxId, 4 , p.VELOCITY_CONTROL, targetVelocity = action[3], force= 250_000)

        #Update Simulations
        p.stepSimulation()
        time.sleep(self.dt)

        #Orientation (Coming Soon)

        #Calculate error
        self.theta_now = self._get_joint_state()
        linkWorldPosition, *_ = p.getLinkState(self.boxId,3, computeLinkVelocity=1, computeForwardKinematics=1)

        vec = np.array(linkWorldPosition) - self.position_target

        reward_dist = -np.linalg.norm(vec)
        reward_ctrl = -np.square(action).sum()

        reward = reward_dist + reward_ctrl
        self.new_obs = self._get_obs(action, vec)

        # if np.any(self.theta_now > np.array(self.max_theta)) or np.any(self.theta_now < np.array(self.min_theta)):
        #     done = True
        #     self.reward = -1000
        # else:
        done = bool(self.steps_left<0)
        self.reward = reward
        self.steps_left -= 1

        #Update State
        
        self.act = action
        self.cur_done = done
        return self.new_obs, self.reward, done, dict(reward_dist=reward_dist, reward_ctrl=reward_ctrl)

    def start_simulation(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

        ## Setup Physics
        p.setGravity(0,0,-9.8)

        ## Load Plane
        planeId = p.loadURDF("plane.urdf")

        ## Load Robot
        startPos = [self.state[0],self.state[1],1.4054411813121799]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.boxId = p.loadURDF("aba_excavator/excavator.urdf",startPos, startOrientation)

    def reset(self):
        p.resetSimulation()
        self.start_simulation()
        self.theta_now = self._get_joint_state()
        self.steps_left = np.copy(self.MAX_EPISODE)
        self.act = [0,0,0,0]
        self.cur_done = False
        self.new_obs = np.zeros(12)
        return self.new_obs

    def render(self, mode='human'):
        print(f'State {self.new_obs}, action: {self.act}, done: {self.cur_done}')

    def _get_joint_state(self):
        theta0, theta1, theta2, theta3 = p.getJointStates(self.boxId, [1,2,3,4])
        return self.normalize(np.array([theta0[0], theta1[0], theta2[0], theta3[0]]))

    def normalize(self, x):
        return ((x+np.pi)%(2*np.pi)) - np.pi

    def _get_obs(self, action, error):
        return np.concatenate(
            [
                np.cos(self.theta_now[1:3]),
                np.sin(self.theta_now[1:3]),
                self.position_target,
                action,
                error
            ]
        )
    



