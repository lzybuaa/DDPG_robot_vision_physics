import pybullet as p
import math
from time import sleep
path= "D:/bullet3-master/data/"   

p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0,0,-9.8)
table=p.loadURDF(path+"table/table.urdf")
kukaId = p.loadURDF(path+"kuka_lwr/kuka.urdf",[0,0,0.6],useFixedBase=True)
kukaJoint=p.getNumJoints(kukaId)
# ballId = p.loadURDF(path+"sphere_small.urdf",[0.3,-0.3,0.7],useFixedBase=True)

endPos,endOrn=p.getLinkState(kukaId,6)[0:2]   # position and orientation of end effector

distance=0.1   # distance from camera to ball
viewMatrix = p.computeViewMatrixFromYawPitchRoll(endPos,distance,0,0,0,2)   # camera doesn't change its view angle, robot does
projectMatrix = p.computeProjectionMatrixFOV(60,1,0.1,100)     # input: field of view, ratio(width/height),near,far
rgbpix,depthpix=p.getCameraImage(512,512,viewMatrix,projectMatrix)[2:4]   # input: image resolution

for i in range (1,10):
    p.stepSimulation()
    sleep(1)