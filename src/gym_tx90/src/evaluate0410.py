'''
Name: 
Date: 2024-04-10 15:17:35
Creator: 王飞鸿
Description: 
'''
import gym
import gym_envs
from stable_baselines3 import PPO # 假设你使用的是PPO算法
import os
from stable_baselines3.common.evaluation import evaluate_policy
load_model_dir = "/home/wfh/桌面/TX_PiH_R/PiH_model/"
env = gym.make('txPiH-v1', render=True)
model = PPO.load(load_model_dir, env=None)

# 评估模型
mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10, deterministic=True, render=False)

print(f"Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")