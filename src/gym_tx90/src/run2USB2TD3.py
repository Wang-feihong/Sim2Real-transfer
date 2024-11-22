'''
Name: 
Date: 2024-04-24 14:41:36
Creator: 王飞鸿
Description: 
'''
import os
import gym
import gym_envs
import argparse
from stable_baselines3 import PPO,TD3,SAC,HerReplayBuffer
from typing import Callable
import torch
from stable_baselines3.common.callbacks import CallbackList, BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.evaluation import evaluate_policy
import numpy as np
from datetime import datetime

def linear_schedule(initial_value: float, lowest_value: float = 0.000) -> Callable[[float], float]:
    def func(progress_remaining: float) -> float:  # 计算当前学习率
        return progress_remaining * initial_value + lowest_value
    return func

c_range = 0.25  # 表示奖励函数中的一个参数，用于控制奖励函数的范围
ent_coef = 0.0016  # 表示策略的熵系数，用于控制策略的探索性
learning_rate = 0.0003 # 0.0003
record_obs = True
policy_kwargs = dict(activation_fn=torch.nn.Tanh,
                     net_arch=dict(pi=[128, 128], vf=[128, 128]))  # 双曲正切函数激活，策略网络（pi）和值函数网络（vf）的隐藏层结构
_clip_range = linear_schedule(c_range)
_learning_rate = linear_schedule(learning_rate, lowest_value=0.0001)
running_time = datetime.now().strftime("%m%d%H%M%S")

callback = CallbackList([])
root_dir_tensorboard = '/home/wfh/PiH-RL/tensorboard_0418/'
root_dir_model ="/home/wfh/桌面/TX_PiH_R/PiH_model/"
recording_path = '/home/wfh/PiH-RL/recording1215/' + running_time

env = gym.make('tx2USB-v1',render=True)
algorithm = 'SAC'
if algorithm == 'TD3':
    model = TD3(
        policy='MultiInputPolicy',  # 策略网络的类型
        env=env,
        buffer_size=500000,
        replay_buffer_class=HerReplayBuffer,
        verbose=1,
        learning_rate=_learning_rate,
        tensorboard_log=root_dir_tensorboard)
elif algorithm == 'SAC':
    model = SAC(policy='MlpPolicy',
                env=env,
                verbose=1,
                buffer_size=100000,
                learning_starts=10000,
                learning_rate=_learning_rate,
            tensorboard_log=root_dir_tensorboard)  # tesorboard日志地址
model.learn(total_timesteps=int(5e5), callback=callback, tb_log_name="txPiH_run_"+ algorithm +'_'+running_time)  # (训练steps)
model.save(root_dir_model+running_time+".pkl")  # 保存
#model = PPO.load(root_dir_model, env=None)
#mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10, deterministic=True, render=False)
#print(f"Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")
env.close()