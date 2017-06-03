from PYBULLET_ROBOT import PybulletRobot as PR
import time
import numpy as np

pr = PR();
time.sleep(1)
print('ready to step...')
pr._step_torque(np.array([0.2,-0.8,1.4,0.1,0,0.3,0.15]))
print('ready to reset...')
pr._reset()
time.sleep(20)