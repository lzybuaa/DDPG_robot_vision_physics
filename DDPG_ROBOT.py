"""
Deep Deterministic Policy Gradient (DDPG), Reinforcement Learning.
DDPG is Actor Critic based algorithm.
6 link robot example

Using:
python 3.5.3
tensorflow 1.1.0
gym 0.9.1
numpy 1.13
"""


import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import tensorflow as tf
import numpy as np
import gym

