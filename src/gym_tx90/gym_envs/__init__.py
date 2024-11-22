'''
Name: 
Date: 2024-01-24 22:24:14
Creator: 王飞鸿
Description: 
'''
from gym.envs.registration import register
#from gymnasium.envs.registration import register
register(
    id="tx2USB-v1",
    entry_point="gym_envs.envs:TX2USBEnv",
)
register(
    id="tx2USB-v2",
    entry_point="gym_envs.envs:TX2USB2Real",
)
