import pybullet as p
import math
from time import sleep
import numpy as np
import os

path = os.getcwd()
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.8)
kukaId = p.loadURDF(path + "\\kuka_lwr\kuka.urdf", (0, 0, 0), useFixedBase=True)
numjoint = p.getNumJoints(kukaId)
ballId = p.loadURDF(path + "\\ball\sphere_small.urdf", (1, -1, 0.7), useFixedBase=True)
p.changeVisualShape(ballId, 0, rgbaColor=(0, 1, 0, 1))  # green ball
#p.changeDynamics(ballId, 0, mass=1.0)

ballpos,ballorn = p.getBasePositionAndOrientation(ballId)
# inverse kinematic
jointinit = p.calculateInverseKinematics(kukaId,numjoint-1,(0,0.2,0.7))
# set initial pos for robot
for i in range (numjoint):
	p.resetJointState(kukaId,i,jointinit[i])
sleep(0.01)

# position and orientation of end effector
endPos,endOrn=p.getLinkState(kukaId,6)[0:2]   
rotmatrix=p.getMatrixFromQuaternion(endOrn)
 # distance from camera to focus
distance=0.1

# where does camera aim at
camview=list()   
for i in range(3):
    camview.append(np.dot(rotmatrix[i*3:i*3+3],(0,0,-distance)))
camview[2]=-camview[2]
tagPos=np.add(camview,endPos)

#endOrn=np.divide(p.getEulerFromQuaternion(endOrn),(2*math.pi))*360
#viewMatrix = p.computeViewMatrixFromYawPitchRoll(endPos,distance,-endOrn[2],-endOrn[1],endOrn[0]+180,2)   # camera doesn't change its view angle, robot does

viewMatrix=p.computeViewMatrix(endPos,tagPos,(0,1,0))
projectMatrix = p.computeProjectionMatrixFOV(60,1,0.1,100)     # input: field of view, ratio(width/height),near,far
rgbpix,depthpix=p.getCameraImage(128,128,viewMatrix,projectMatrix)[2:4]   # input: image resolution

# free fall ball
for i in range(1000):   
    p.stepSimulation()
    p.setRealTimeSimulation(1)
    p.applyExternalForce(ballId,-1,(0,0,-9.8),(0,0,0),flags=2)
    ballPos,ballOrn=p.getBasePositionAndOrientation(ballId)

for i in range (1,10):
    p.stepSimulation()
    sleep(0.01)
