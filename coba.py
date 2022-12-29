import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
 
planeId = p.loadURDF("plane.urdf")
 
startPos = [0, 0, 1.4054411813121799]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("aba_excavator/excavator.urdf",startPos, startOrientation)
o,o1 = [], []

def normalize(x):
    return ((x+np.pi)%(2*np.pi)) - np.pi

for i in range(1000):
    p.setJointMotorControl2(boxId, 1 , p.POSITION_CONTROL, targetPosition = 0, force=50000)
    p.setJointMotorControl2(boxId, 2 , p.POSITION_CONTROL, targetPosition = -0.5, force= 250_000)
    p.setJointMotorControl2(boxId, 3 , p.POSITION_CONTROL, targetPosition = 1, force= 250_000)
    p.setJointMotorControl2(boxId, 4 , p.VELOCITY_CONTROL, targetVelocity = np.pi, force=250_000)
    (linkWorldPosition, orientation, *_) = p.getLinkState(boxId,5, computeLinkVelocity=1, computeForwardKinematics=1)
    # print(linkWorldPosition)    
    p.stepSimulation()
    time.sleep(1.0/240.)
    theta1 = p.getJointState(boxId,2)
    theta2 = p.getJointState(boxId,3)
    theta3 = p.getJointState(boxId,4)
    print(linkWorldPosition)
    ori = normalize(-(theta1[0]+theta2[0]+theta3[0]))
    orientation = p.getEulerFromQuaternion(orientation)
    print(orientation[1], ori)
    o.append(np.array(orientation))
    o1.append(ori)
    if ori==-np.pi:
        break
# p.disconnect()
plt.plot(list(range(1000)),np.array(o)[:,1])
plt.plot(list(range(1000)),np.array(o1))
plt.show()
