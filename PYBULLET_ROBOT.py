'''
This class is written as interface to the pybullet physical environment
'''

import numpy as np
import pybullet as p
import time
#from VISIONREWARD import VisionReward

state_space_init = np.array([0,0,0,0,0,0,\
			                 0,0,0,0,0,0])   # hardcode it later
action_space_init = np.array([0,0,0,0,0,0])
robot_orn_init = [0,0,0]   # siwei should hardcode this
robot_pos_init = [0,0,3]

class PybulletRobot:

	def __init__(self, robot_path, ball_path, ball_color):
		# state space includes joint angles and velocities
		self.state_space = state_space_init
		# action space are acceleration/torques for each joint
		self.action_space = action_space_init
		p.connect(p.GUI)
		p.resetSimulation()
		self.robot_id = p.loadURDF(robot_path,[0,0,0],useFixedBase=True)
		self.ball_id = p.loadURDF(ball_path,[0,0,0],useFixedBase=True)
		self.robot_joint_num = p.getNumJoints(self.robot_id)
		self.ball_color = p.changeVisualShape(self.ball_id, 0, rgbaColor=ball_color)
		p.setGravity(0,0,-9.8)
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

	def _reset_robot():
		# move back to the initial position


	def _reset_ball():

	def _reset(self):
		# restore the state space to original ones
		self.state_space = state_space_init
		self.action_space = action_space_init
		# restore to the initial position

		

	def _step(self):
		orn_init = p.getQuaternionFromEuler([0,-np.pi,0])
		for j in range(10):
			end_pos_init = p.calculateInverseKinematics(self.robot_id,endEffectorLinkIndex=6,targetPosition=[0.3,0.3,0.3],targetOrientation=orn_init,jointDamping=[0.1,0.1,0.1,0.1,0.1,0.1,0.1])
			for i in range(self.robot_joint_num):
				p.setJointMotorControl2(self.robot_id,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=end_pos_init[i])
				p.stepSimulation()
				time.sleep(0.01)
		print('finished!')
		time.sleep(20)



