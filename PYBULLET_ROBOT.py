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
state_space_init = np.array([0,0,0]) # x, y, r
action_space_init = np.array([0,0,0,0,0,0,0])
robot_orn_init = [0,0,0]   # siwei should hardcode this
robot_pos_init = [0,0,0]
ball_pos_init = [0.3,0.3,2]
ground_pos = [0,0,0]
j_d = [0.1,0.1,0.1,0.1,0.1,0.1,0.1]  # joint damping



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
		p.setRealTimeSimulation(1)

		# action low and high mapped to 1
		self.action_low = 200
		self.action_high = 800

		# weights
		self.center_reward_weight = 1
		self.radius_reward_weigth = 2

		# load the two items
		self.robot_id = p.loadURDF(robot_path, robot_pos_init, p.getQuaternionFromEuler(robot_orn_init), useFixedBase=True)
		self.ball_id = p.loadURDF(ball_path, ball_pos_init)
		self.ground_id = p.loadURDF(ground_path, ground_pos, useFixedBase=True)

		# get the joint number and the initial joint state
		self.robot_joint_num = p.getNumJoints(self.robot_id)
		#self.init_joint_state = []  # obsolete because resetJointState doesn't work with setRealTimeSimulation
		#for i in range(self.robot_joint_num):
			#self.init_joint_state.append(p.getJointState(self.robot_id, i)[0])
		
	
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
		c_action = np.clip(action, -1, 1)
		return self.action_low * np.sign(c_action) + (self.action_high - self.action_low) * np.array(c_action)

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
		print('moving back to the original position')
		for j in range(20):
			end_pos_init = p.calculateInverseKinematics(self.robot_id,6,[0,0,3],p.getQuaternionFromEuler(robot_orn_init),jointDamping=j_d)
			#for i in range(self.robot_joint_num):
			p.setJointMotorControlArray(self.robot_id,np.arange(self.robot_joint_num),p.POSITION_CONTROL,targetPositions=end_pos_init)
			#p.stepSimulation()
			time.sleep(0.001)
		time.sleep(0.5)
		# reset the ball's position
		print('ball back to the original position')
		p.resetBasePositionAndOrientation(self.ball_id, ball_pos_init, p.getQuaternionFromEuler(robot_orn_init))
		# take the picture before ball moves
		#s = self._take_picture()
		p.setGravity(0,0,-9.8)
		#return s

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
			end_pos_init = p.calculateInverseKinematics(self.robot_id,6,position,orn_init,jointDamping=j_d)
			#for i in range(self.robot_joint_num):
			p.setJointMotorControlArray(self.robot_id,np.arange(self.robot_joint_num),p.POSITION_CONTROL,targetPositions=end_pos_init)
			#p.stepSimulation()
			time.sleep(0.001)
		print('finished!')

	# step by given action (torque)
	def _step(self, action):
		mapped_action = self._map_action(action)
		print(mapped_action)
		#mapped_action = action
		for j in range(200):
			p.setJointMotorControlArray(self.robot_id,np.arange(self.robot_joint_num),p.TORQUE_CONTROL,forces=mapped_action)
			#p.stepSimulation()
			time.sleep(0.001)
		#r = self._compute_reward(self._take_picture())
		#return (self.state_space, r)


	# perform taking pictures
	def _take_picture(self):
		# get the last link's position and orientation
		Pos, Orn = p.getLinkState(self.robot_id, self.robot_joint_num)[:2]
		# Pos is the position of end effect, orn is the orientation of the end effect
		rotmatrix = p.getMatrixFromQuaternion(Orn)
		# distance from camera to focus
		distance = 0.2
		# where does camera aim at
		camview = list()
		for i in range(3):
			camview.append(np.dot(rotmatrix[i*3:i*3+3], (0, 0, distance)))
		tagPos = np.add(camview,Pos)
		#p.removeBody(kukaId)
		viewMatrix = p.computeViewMatrix(Pos, tagPos, (0, 0, 1))
		#viewMatrix=p.computeViewMatrix([-2, 0, 2], [0, 0, 1],[0, 0, 1])
		projectMatrix = p.computeProjectionMatrixFOV(60, 1, 0.1, 100)     # input: field of view, ratio(width/height),near,far
		rgbpix = p.getCameraImage(128, 128, viewMatrix, projectMatrix)[2]
		return rgbpix[:, :, 0:3]


	def _update_center_and_size(self, image_frame):
		# compute the center and radius of image
	    """ Function to extract info from an image
	    Takes the image_frame array and sends it into the opencv algorithm.
	    Returns the center and size of any COLOR_BALL (defined in the constant) that it
	    detects in the image.
	    :param image_frame:
	    :return: Return is in the format tuple ((x, y), radius)
	    Error conditions:
	        if no ball is detected, then return (None, 0)
	    """

	    # preprocessing
	    # we won't downsize, since the image is modifyable already
	    # image_frame = imutils.resize(image_frame, width=DOWNSIZE)
	    hsv = cv2.cvtColor(image_frame, cv2.COLOR_BGR2HSV)

	    # mask
	    color = np.array(BALL_COLOR)
	    mask = cv2.inRange(hsv, greenLower, greenUpper)
	    # don't need the erosion/dilation b/c sim images are perfect
	    # mask = cv2.erode(mask, None, iterations=2)
	    # mask = cv2.dilate(mask, None, iterations=2)

	    # find contours in mask and initialize current center of ball
	    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	                            cv2.CHAIN_APPROX_SIMPLE)[-2]
	    center = None
	    radius = 0

	    # only proceed if at least one contour was found
	    if len(cnts) > 0:
	        # find the largest contour in the mask, then use
	        # it to compute the minimum enclosing circle and
	        # centroid
	        c = max(cnts, key=cv2.contourArea)
	        ((x, y), radius) = cv2.minEnclosingCircle(c)
	        M = cv2.moments(c)
	        center = [int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])]

	    # update the state space
	    self.state_space[0:2] = center[0:2]
	    self.state_space[2] = radius



	def _compute_reward(self, image):
		# given the image, return the functions
		dim = image.shape
		self._update_center_and_size(image)
		#return 1 / ()




