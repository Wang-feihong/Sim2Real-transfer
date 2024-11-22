'''
Name:
Date: 2024-03-12 18:57:15
Creator: 王飞鸿
Description:
'''
import gym
import gym_envs
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
import torch

load_model_dir = "/home/wfh/桌面/TX_PiH_R/PiH_model/"
env = gym.make("txPiH-v1", render=True)
model = PPO.load(load_model_dir,env=env)
# mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
#评估
mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
print(f"Mean reward = {mean_reward:.2f} Std_reward =  {std_reward:.2f}")
episodes = 10
score = 0
obs = env.reset()
for i in range(5000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, _, info = env.step(action)
    score += reward
    env.render()
print("score=",score)

