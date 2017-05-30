'''
This class is written as interface to the pybullet physical environment
'''

import numpy as np
import pybullet as p
import time as t
#from VISIONREWARD import VisionReward

state_space_init = np.array([0,0,0,0,0,0,\
			                 0,0,0,0,0,0])   # hardcode it later
action_space_init = np.array([0,0,0,0,0,0])

class PybulletRobot:

	def __init__(self, robot_path, ball_path, ball_color):
		# state space includes joint angles and velocities
		self.state_space = state_space_init
		# action space are acceleration/torques for each joint
		self.action_space = action_space_init
		self.robot_id = p.loadURDF(robot_path,[0,0,0],useFixedBase=True)
		self.ball_id = p.loadURDF(ball_path,[0,0,0],useFixedBase=True)
		self.robot_joint_num = p.getNumJoints(self.robot_id)
		self.ball_color = p.changeVisualShape(self.ball_id, 0, rgbcolor=ball_color)

	def _init():
		p.resetSimulation()
		p.setGravity(0,0,-9.8)
		

	
	def _reset():
		self.state_space = state_space_init
		self.action_space = action_space_init

	def _step():



