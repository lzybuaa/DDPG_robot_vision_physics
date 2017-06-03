from PYBULLET_ROBOT import PybulletRobot as PR
import time
import numpy as np

pr = PR();
time.sleep(3)
print('simulation ready...')
pr._reset()
#for j in range(20):
action = [0.6,0,0,0,0,0,0]#np.random.uniform(-1,1,7)
pos = [0.2,0.3,0.4]
orn = [0, -np.pi, 0]
pr._step(action)
print('simulation finished!')
time.sleep(5)