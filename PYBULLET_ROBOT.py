'''
This class is written as interface to the pybullet physical environment
'''

import numpy as np
import pybullet as p
import time
import os

# get the current directory
path = os.getcwd()
robot_path = path+'\\kuka_lwr\\kuka.urdf'
ball_path = path+'\\ball\sphere_small.urdf'

# parameters setup
state_space_init = np.array([0,0,0,0,0,0,\
			                 0,0,0,0,0,0])   # hardcode it later
action_space_init = np.array([0,0,0,0,0,0])
robot_orn_init = [0,0,0]   # siwei should hardcode this
robot_pos_init = [0,0,0]
ball_pos_init = [0.3,0.3,2]



class PybulletRobot:

	def __init__(self):
		# state space includes joint angles and velocities
		self.state_space = state_space_init
		# action space are acceleration/torques for each joint
		self.action_space = action_space_init
		self.frame_rate = 30
		p.connect(p.GUI)
		p.resetSimulation()
		p.setGravity(0,0,-9.8)
		self.robot_id = p.loadURDF(robot_path,robot_pos_init,p.getQuaternionFromEuler(robot_orn_init),useFixedBase=True)
		self.ball_id = p.loadURDF(ball_path,ball_pos_init)
		time.sleep(2)
	
	def _state_space_dim(self):
		try:
			return self.state_space.shape[0]
		except:
			print('state space is none. check your values!')
			return -1

	def _action_space_dim(self):
		try:
			return self.action_space.shape[0]
		except:
			print('action space is none. check your values!')
			return -1

	def _reset(self):
		# restore the state space to original ones
		self.state_space = state_space_init
		self.action_space = action_space_init
		# reset the two initial positon and orientation
		p.resetBasePositionAndOrientation(self.robot_id, robot_pos_init, p.getQuaternionFromEuler(robot_orn_init))
		p.resetBasePositionAndOrientation(self.ball_id, ball_pos_init, p.getQuaternionFromEuler(robot_orn_init))

	def _step(self):
		p.setGravity(0,0,-9.8)
		orn_init = p.getQuaternionFromEuler([0,-np.pi,0])
		for j in range(10):
			end_pos_init = p.calculateInverseKinematics(self.robot_id,endEffectorLinkIndex=6,targetPosition=[0.3,0.3,0.3],targetOrientation=orn_init,jointDamping=[0.1,0.1,0.1,0.1,0.1,0.1,0.1])
			for i in range(self.robot_joint_num):
				p.setJointMotorControl2(self.robot_id,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=end_pos_init[i])
				p.stepSimulation()
				time.sleep(0.01)
		print('finished!')
		time.sleep(20)

	def _computeCenterAndSize(image):
		return (center, size)

	def _computeReward(center, size):
		# given the image, return the 
		return reward




