import pybullet as p
import math
from time import sleep
import numpy as np
path= "D:/bullet3-master/data/"   

p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0,0,-9.8)
table=p.loadURDF(path+"table/table.urdf")
kukaId = p.loadURDF(path+"kuka_lwr/kuka.urdf",[0,0,0.6],useFixedBase=True)
kukaJoint=p.getNumJoints(kukaId)
ballId = p.loadURDF(path+"sphere_small.urdf",[0.3,-0.3,2.5],useFixedBase=True)
p.changeVisualShape(ballId,0,rgbaColor=(0,1,0,1))  # green ball

for i in range(1000):   # make ball free fall
    p.stepSimulation()
    p.setRealTimeSimulation(1)
    p.applyExternalForce(ballId,-1,(0,0,-9.8),(0,0,0),flags=2)

endPos,endOrn=p.getLinkState(kukaId,6)[0:2]   # position and orientation of end effector
rotmatrix=p.getMatrixFromQuaternion(endOrn)
distance=0.1   # distance from camera to focus

camview=list()   # where does camera aim at
for i in range(3):
    camview.append(np.dot(rotmatrix[i*3:i*3+3],(0,0,-distance)))
camview[2]=-camview[2]
tagPos=np.add(camview,endPos)
#endOrn=np.divide(p.getEulerFromQuaternion(endOrn),(2*math.pi))*360
#viewMatrix = p.computeViewMatrixFromYawPitchRoll(endPos,distance,-endOrn[2],-endOrn[1],endOrn[0]+180,2)   # camera doesn't change its view angle, robot does
viewMatrix=p.computeViewMatrix(endPos,tagPos,(0,1,0))
projectMatrix = p.computeProjectionMatrixFOV(60,1,0.1,100)     # input: field of view, ratio(width/height),near,far
rgbpix,depthpix=p.getCameraImage(512,512,viewMatrix,projectMatrix)[2:4]   # input: image resolution

for i in range (1,10):
    p.stepSimulation()
    sleep(1)

