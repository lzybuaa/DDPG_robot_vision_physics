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
ground_path = path + "\\floor\plane100.urdf"

# parameters setup
state_space_init = np.array([0,0,0,0,0,0,0,\
			                 0,0,0,0,0,0,0])   # hardcode it later
action_space_init = np.array([0,0,0,0,0,0,0])
robot_orn_init = [0,0,0]   # siwei should hardcode this
robot_pos_init = [0,0,0]
ball_pos_init = [0.3,0.3,2]
ground_pos = [0,0,0]
j_d = [0.1,0.1,0.1,0.1,0.1,0.1,0.1]



class PybulletRobot:

	def __init__(self):

		# state space - joint angles and velocities action space - acceleration/torques for each joint
		self.state_space = state_space_init
		self.action_space = action_space_init

		# camera frame rate
		self.frame_rate = 30

		# inistialize pybullet physics environment
		p.connect(p.GUI)
		p.resetSimulation()
		p.setGravity(0,0,-9.8)
		p.setRealTimeSimulation(1)

		# action low and high mapped to 1
		self.action_low = 200
		self.action_high = 500

		# load the two items
		self.robot_id = p.loadURDF(robot_path, robot_pos_init, p.getQuaternionFromEuler(robot_orn_init), useFixedBase=True)
		self.ball_id = p.loadURDF(ball_path, ball_pos_init)
		self.ground_id = p.loadURDF(ground_path, ground_pos, useFixedBase=True)

		# get the joint number and the initial joint state
		self.robot_joint_num = p.getNumJoints(self.robot_id)
		#self.init_joint_state = []  # obsolete because resetJointState doesn't work with setRealTimeSimulation
		#for i in range(self.robot_joint_num):
			#self.init_joint_state.append(p.getJointState(self.robot_id, i)[0])
		
		# sleep before action
		#time.sleep(2)
	
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

	def _get_state(self):
		joint_state = [] 
		for i in range(self.robot_joint_num):
			self.init_joint_state.append(p.getJointState(self.robot_id, i)[0])
		return joint_state

	# maps (-1, 0) to (-500, -200), (0, 1) to (200, 500)
	def _map_action(self, action):
		return self.action_low * np.sign(action) + (self.action_high - self.action_low) * np.array(action)

	# reset the robot to the initial state
	def _reset(self):
		# restore the state space to original ones
		self.state_space = state_space_init
		self.action_space = action_space_init
		#p.setRealTimeSimulation(0)
		p.setGravity(0,0,0)
		# reset the robot's joint states
		#for i in range(self.robot_joint_num):
			#p.resetJointState(self.robot_id, i, self.init_joint_state[i])
		for j in range(20):
			end_pos_init = p.calculateInverseKinematics(self.robot_id,6,[0,0,3],p.getQuaternionFromEuler(robot_orn_init),jointDamping=[0.1,0.1,0.1,0.1,0.1,0.1,0.1])
			#for i in range(self.robot_joint_num):
			p.setJointMotorControlArray(self.robot_id,np.arange(self.robot_joint_num),p.POSITION_CONTROL,targetPositions=end_pos_init)
			#p.stepSimulation()
			time.sleep(0.001)
		time.sleep(1)
		# reset the ball's position
		p.resetBasePositionAndOrientation(self.ball_id, ball_pos_init, p.getQuaternionFromEuler(robot_orn_init))
		p.setGravity(0,0,-9.8)

	# check if ball collide with the ground
	def _check_coliision(self):
		if len(p.getContactPoints(self.ball_id, self.ground_id)) is 0:
			return False
		else:
			return True

	# step by postion
	def _step_pos(self, position, orientation):
		orn_init = p.getQuaternionFromEuler(orientation)
		for j in range(100):
			end_pos_init = p.calculateInverseKinematics(self.robot_id,6,position,orn_init,jointDamping=[0.1,0.1,0.1,0.1,0.1,0.1,0.1])
			#for i in range(self.robot_joint_num):
			p.setJointMotorControlArray(self.robot_id,np.arange(self.robot_joint_num),p.POSITION_CONTROL,targetPositions=end_pos_init)
			#p.stepSimulation()
			time.sleep(0.001)
		print('finished!')

	# step by given action (torque)
	def _step_torque(self, action):
		for j in range(10):
			p.setJointMotorControlArray(self.robot_id,np.arange(self.robot_joint_num),p.TORQUE_CONTROL,forces=action)
			#p.stepSimulation()
			time.sleep(0.01)

	def _computeCenterAndSize(image):
		return (center, size)

	def _computeReward(center, size):
		# given the image, return the 
		return reward




