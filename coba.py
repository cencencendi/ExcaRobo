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

pose_target = np.array([[0.7,-1.4,0],
                        [0.144,-0.59,-1.47],
                        [0.257,-1.17,-1.19],
                        [0.294,-1.437,-1.823],
                        [0.444,-1.4588,-1.859]])
def normalize(x):
    return ((x+np.pi)%(2*np.pi)) - np.pi

for pose in pose_target:
    print(pose)
    for i in range(200):
        p.setJointMotorControl2(boxId, 1 , p.POSITION_CONTROL, targetPosition = 0, force=50000)
        p.setJointMotorControl2(boxId, 2 , p.POSITION_CONTROL, targetPosition = -pose[0], force= 250_000)
        p.setJointMotorControl2(boxId, 3 , p.POSITION_CONTROL, targetPosition = -pose[1], force= 250_000)
        p.setJointMotorControl2(boxId, 4 , p.POSITION_CONTROL, targetPosition = -pose[2], force=250_000)
        (linkWorldPosition, orientation, *_) = p.getLinkState(boxId,4, computeLinkVelocity=1, computeForwardKinematics=1)
        # print(linkWorldPosition)    
        p.stepSimulation()
        time.sleep(1.0/240.)
        # theta1 = p.getJointState(boxId,2)
        # theta2 = p.getJointState(boxId,3)
        # theta3 = p.getJointState(boxId,4)
        # print(linkWorldPosition)
        # ori = normalize(-(theta1[0]+theta2[0]+theta3[0]))
        # orientation = p.getEulerFromQuaternion(orientation)
        # print(orientation[1], ori)
        o.append(np.array(linkWorldPosition))
        # o1.append(ori)

p.disconnect()
o = np.array(o)
plt.plot(o[:,0],o[:,2])
# plt.plot(list(range(1000)),np.array(o1))
# plt.show()
