'''
Name: 
Date: 2024-05-01 09:29
Creator: 王飞鸿
Description: 评估USB插孔准确率
'''
import gym
#import gymnasium as gym
import gym_envs
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_checker import check_env
import torch


load_model_dir = "/home/wfh/桌面/TX_PiH_R/PiH_model/"
env = gym.make("tx2USB-v1", render=True)
#check_env(env)
# model = PPO.load(load_model_dir,env=env)  # 共享策略

# 评估模型
#mean_reward, std_reward = evaluate_policy(model, model.get_env, n_eval_episodes=10)
# valuate_policy(model, env, n_eval_episodes=10, deterministic=True, render=False, callback=None, reward_threshold=None, return_episode_rewards=False, warn=True)
episodes = 100
total_attempts = 0  # 记录总的尝试次数
#obs = env.reset()
success_count = 0  # 记录成功插入孔的次数
total_score = 0
for episode in range(episodes):
    obs, info = env.reset()
    done = False
    score = 0
    # total_attempts = 0  # 记录总的尝试次数
    cur_attempts = 0
    while not done:
        action, _states = model.predict(obs, deterministic=True) #deterministic=True
        obs, reward, done, _, info = env.step(action)
        score += reward
        #env.render()
        if 'is_success' in info and info['is_success']:
            #print("info:", info)
            success_count += 1
            done = True
        cur_attempts += 1  # 记录总的尝试次数

    total_attempts += cur_attempts
    total_score += score
    print("Episode:", episode + 1, "Score:", score , 'success_count:', success_count, 'cur_attempts:', cur_attempts)
average_score = total_score / episodes
average_attempts = total_attempts/episodes
success_rate = success_count / episodes
print("成功率 = ", success_rate, "平均步数 = ", average_attempts, "平均奖励 = ", average_score)
