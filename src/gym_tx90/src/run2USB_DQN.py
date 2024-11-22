#!/usr/bin/env python3
#coding=utf-8

import os
import gym
import gym_envs
import argparse
from stable_baselines3 import PPO,TD3,SAC,HerReplayBuffer,DQN
from typing import Callable
import torch
from stable_baselines3.common.callbacks import CallbackList, BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.evaluation import evaluate_policy
import numpy as np
from datetime import datetime

# import rospy
# from std_msgs.msg import String

# if __name__ == "__main__":
#     rospy.init_node("nodepy1")
#     rospy.logwarn("rospy登场！")

def linear_schedule(initial_value: float, lowest_value: float = 0.000) -> Callable[[float], float]:
    def func(progress_remaining: float) -> float:  # 计算当前学习率
        return progress_remaining * initial_value + lowest_value
    return func

c_range = 0.25  # 表示奖励函数中的一个参数，用于控制奖励函数的范围
ent_coef = 0.0016  # 表示策略的熵系数，用于控制策略的探索性
learning_rate = 0.0003 # 0.0003
record_obs = True
_clip_range = linear_schedule(c_range)
_learning_rate = linear_schedule(learning_rate, lowest_value=0.0001)
running_time = datetime.now().strftime("%m%d%H%M%S")

callback = CallbackList([])
root_dir_tensorboard = '/home/wfh/PiH-RL/tensorboard_ROS_DQN/'
root_dir_model ="/home/wfh/catkin_ws/src/gym_tx90/PiH_model/"
recording_path = '/home/wfh/PiH-RL/recording_DQN/' + running_time

env = gym.make('tx2USB-v1',render=True)
algorithm = "DQN"
model = DQN(
    policy='MlpPolicy',  # 策略网络的类型
    env=env,
    buffer_size=10000,
    device='cuda:0',
    # replay_buffer_class=HerReplayBuffer,
    verbose=1,
    learning_rate=_learning_rate,
    tensorboard_log=root_dir_tensorboard)
model.learn(total_timesteps=int(1.5e5), callback=callback, tb_log_name="txUSB_"+ algorithm +'_'+running_time)  # (训练steps)
model.save(root_dir_model+running_time+".pkl")  # 保存
#model = PPO.load(root_dir_model, env=None)
#mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10, deterministic=True, render=False)
#print(f"Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")
env.close()