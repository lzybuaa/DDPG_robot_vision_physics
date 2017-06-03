from PYBULLET_ROBOT import PybulletRobot as PR
import time
import numpy as np

pr = PR()
time.sleep(3)
'''
print('simulation ready...')
for i in range(5):
	pr._reset()
	for j in range(10):
		action = np.random.normal(-1,1,7)
		pr._step(action)
		time.sleep(0.001)
print('simulation finished!')
'''
time.sleep(5)