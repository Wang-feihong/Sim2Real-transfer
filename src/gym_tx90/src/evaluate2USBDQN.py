'''
Name: 
Date: 2024-05-15 22:22:06
Creator: 王飞鸿
Description: 
'''
from datetime import datetime
import os
import gym
import numpy as np
#import gymnasium as gym
import gym_envs
from stable_baselines3 import PPO,DQN
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.evaluation import evaluate_policy  
import torch
def mkdir(path):
    if not os.path.exists(path):
        os.makedirs(path)
running_time = datetime.now().strftime("%m%d%H%M%S")
algorithm = "DQN" #"DDPG" "SAC" "A2C"

load_model_dir = "/home/wfh/catkin_ws/src/gym_tx90/PiH_model/"
env = gym.make("tx2USB-v1", render=True)

model = DQN.load(load_model_dir+algorithm_model,env=env) 

recording_path = '/home/wfh/PiH-RL/recording_DQN/'+ f"{algorithm_model[0:8]}"+ algorithm  
if not os.path.exists(recording_path):
    mkdir(recording_path)  # 先创建文件夹，避免在循环中重复创建
episodes = 20
total_attempts = 0  # 记录总的尝试次数
#obs = env.reset()
success_count = 0  # 记录成功插入孔的次数
record_obs = True  # 记录观测值
total_score = 0
for episode in range(episodes):
    obs, info = env.reset()
    done = False
    score = 0
    cur_attempts = 0
    observation_shape = obs.shape
    obs_record = [np.zeros(observation_shape)]
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        action = int(action)
        print("动作：",action)
        obs, reward, done, _, info = env.step(action)
        obs_record = np.r_[obs_record, [obs]]
        score += reward
        #env.render()
        if 'is_success' in info and info['is_success']:
            #print("info:", info)
            success_count += 1
            done = True
        cur_attempts += 1  # 记录总的尝试次数
    if record_obs:
        # 不需要在循环内部创建文件夹，已经在之前创建过了
        np.save(os.path.join(recording_path, f"{episode+1}.npy"), np.array(obs_record))  # 保存为NumPy数组

    total_attempts += cur_attempts
    total_score += score
    print("Episode:", episode + 1, "Score:", score , 'success_count:', success_count, 'cur_attempts:', cur_attempts+1)
average_score = total_score / episodes
average_attempts = total_attempts/episodes
success_rate = success_count / episodes
print("成功率 = ", success_rate, "平均步数 = ", average_attempts, "平均奖励 = ", average_score)