'''
Name: 
Date: 2024-04-09 16:57:47
Creator: 王飞鸿
Description: 
'''
import gym
#import gymnasium as gym
import gym_envs
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_checker import check_env
import torch


load_model_dir = "/home/wfh/桌面/TX_PiH_R/PiH_model/"
env = gym.make("txPiH-v1", render=True)
#check_env(env)
model = PPO.load(load_model_dir,env=env)
# 评估模型
#mean_reward, std_reward = evaluate_policy(model, model.get_env, n_eval_episodes=10)
# valuate_policy(model, env, n_eval_episodes=10, deterministic=True, render=False, callback=None, reward_threshold=None, return_episode_rewards=False, warn=True)
episodes = 10
#obs = env.reset()
success_count = 0  # 记录成功插入孔的次数
for episode in range(episodes):
    obs, info = env.reset()
    done = False
    score = 0
    total_attempts = 0  # 记录总的尝试次数
    while not done:
        action, _states = model.predict(obs, deterministic=True) #deterministic=True
        obs, reward, done, _, info = env.step(action)
        score += reward
        #env.render()
        if 'is_success' in info and info['is_success']:
            #print("info:", info)
            success_count += 1
            done = True

        total_attempts += 1

    print("Episode:", episode + 1, "Score:", score , 'success_count:', success_count, 'total_attempts:', total_attempts)

success_rate = success_count / episodes
print("Success Rate = success_count / episodes = ", success_rate)
