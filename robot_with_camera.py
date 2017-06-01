import math
import os
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p


path = os.getcwd()
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.8)
kukaId = p.loadURDF(path + "\\kuka_lwr\kuka.urdf", (0, 0, 0), useFixedBase=True)
groundId = p.loadURDF(path + "\\floor\plane100.urdf", (0, 0, 0), useFixedBase=True)
numjoint = p.getNumJoints(kukaId)
ballId = p.loadURDF(path + "\\ball\sphere_small.urdf", (1.5, 0, 0.9), useFixedBase=True)  # free fall after robot setup the initial position
#p.changeVisualShape(ballId, 0, rgbaColor=(0, 255, 0, 1))  # green ball
#p.changeDynamics(ballId, 0, mass=1.0)

#plate = p.loadURDF("D:/bullet3-master/data/dinnerware/plate.urdf", (1, 1, 0.5), useFixedBase=True)
#pan = p.loadURDF("D:/bullet3-master/data/dinnerware/pan_tefal.urdf", (-1, 1, 0.5), useFixedBase=True)
#duck = p.loadURDF("D:/bullet3-master/data/duck_vhacd.urdf", (-1, -1, 0.5), useFixedBase=True)
#teddy = p.loadURDF("D:/bullet3-master/data/teddy_vhacd.urdf", (1, -1, 0.5), useFixedBase=True)
#traybox = p.loadURDF("D:/bullet3-master/data/tray/traybox.urdf", (1, 0, 0.5), useFixedBase=True)
#jenga = p.loadURDF("D:/bullet3-master/data/jenga/jenga.urdf", (0, 1, 0.5), useFixedBase=True)
#iiwa = p.loadURDF("D:/bullet3-master/data/kuka_iiwa/model.urdf", (-1, 0, 0.5), useFixedBase=True)
#lego = p.loadURDF("D:/bullet3-master/data/lego/lego.urdf", (0, -1, 0.5), useFixedBase=True)


for i in range(1,20):
    p.stepSimulation()

ballpos, ballorn = p.getBasePositionAndOrientation(ballId)
initpos = (0.5, 0, 0.9)
initorn = ([0, math.pi/2, 0])
# set initial pos for robot
jd = [10, 10, 10, 10, 1, 1, 0.1]
for j in range(5000):
    p.stepSimulation()
    for i in range (numjoint):
        # inverse kinematic
        jointinit = p.calculateInverseKinematics(kukaId, 6, initpos, initorn, jointDamping=jd)
        p.setJointMotorControl2(bodyIndex = kukaId, jointIndex = i, controlMode = p.POSITION_CONTROL, targetPosition = jointinit[i], targetVelocity = 0, force = 500, positionGain = 0.03, velocityGain = 1)
        #p.resetJointState(kukaId,i,jointinit[i])

# position and orientation of end effector
endPos,endOrn=p.getLinkState(kukaId,6)[0:2]   
rotmatrix=p.getMatrixFromQuaternion(endOrn)
 # distance from camera to focus
distance=0.2
# where does camera aim at
camview=list()   
for i in range(3):
    camview.append(np.dot(rotmatrix[i*3:i*3+3],(0,0,distance)))
camview[2]=camview[2]
tagPos=np.add(camview,endPos)

#endOrn=np.divide(p.getEulerFromQuaternion(endOrn),(2*math.pi))*360
#viewMatrix = p.computeViewMatrixFromYawPitchRoll(endPos,distance,-endOrn[2],-endOrn[1],endOrn[0]+180,2)   # camera doesn't change its view angle, robot does

#p.removeBody(kukaId)
viewMatrix=p.computeViewMatrix(endPos,tagPos,(0,0,1))
#viewMatrix=p.computeViewMatrix([-2, 0, 2], [0, 0, 1],[0, 0, 1])
projectMatrix = p.computeProjectionMatrixFOV(60,1,0.1,100)     # input: field of view, ratio(width/height),near,far
rgbpix=p.getCameraImage(512,512,viewMatrix,projectMatrix)   # input: image resolution


imgplot = plt.imshow(np.reshape(np.array(rgbpix[2])/255.0, (512, 512, 4)))
plt.show()

for i in range (1,10):
    p.stepSimulation()
    sleep(0.01)
