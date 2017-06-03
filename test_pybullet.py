from PYBULLET_ROBOT import PybulletRobot as PR
import time
import numpy as np

pr = PR()
print('simulation starting...')
time.sleep(2)
for i in range(3):
    s = pr._reset()
    ep_reward = 0

    while True:

    	# ending conditions
    	if pr._check_collision():
    		print('collison detected, episode again, the reward is: ', ep_reward)
    		time.sleep(1)
    		break
    	
    	s_, r = pr._step(np.random.normal(-1,1,7))
    	print("the next state is: %s, and the reward is: %f", s_, r)
    	ep_reward += r  # aggregate the episode reward
