'''
Name:
Date: 2024-04-28 19:53:07
Creator: 王飞鸿
Description:USB->构建策略、值网络
'''
import os
import gym
import gym_envs
from datetime import datetime
from typing import Callable, Dict, List, Optional, Tuple, Type, Union

from gymnasium import spaces
import torch as th
from torch import nn

from stable_baselines3 import PPO,TD3,SAC,HerReplayBuffer
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.callbacks import CallbackList, BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class CustomNetwork(nn.Module):
    """
     用于策略和价值函数的自定义网络
     它接收特征提取器提取的特征作为输入
     feature_dim: 使用 features_extractor 提取的特征的维度
     last_layer_dim_pi: (int) 策略网络最后一层的单元数
     last_layer_dim_vf: (int) 价值网络最后一层的单元数
    """
    def __init__(
        self,
        feature_dim: int,
        last_layer_dim_pi: int = 32,
        last_layer_dim_vf: int = 32,
    ):
        super().__init__()

        # IMPORTANT:
        # 保存输出维度，用于创建分布
        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf
        """
        # 策略网络A
        self.policy_net = nn.Sequential(
            # nn.Linear(feature_dim, last_layer_dim_pi), nn.ReLU()
            nn.Linear(feature_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 6),
            nn.Tanh(),
        )
        # 值网络A
        self.value_net = nn.Sequential(
            nn.Linear(feature_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 6),
            nn.Tanh()
        )
        """
        # 策略网络B 增加网络深度和宽度、提供更强的表达和学习能力 ReLU更好地处理非线性和梯度传播
        self.policy_net = nn.Sequential(
            nn.Linear(feature_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, self.latent_dim_pi),
            nn.Tanh()
        )

        # 值网络B
        self.value_net = nn.Sequential(
            nn.Linear(feature_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, self.latent_dim_vf),
            nn.Tanh()
        )
    def forward(self, features: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        """
        返回：指定网络的 latent_policy、latent_value(th.Tensor、th.Tensor)
        如果所有层都是共享的,则latent_policy == latent_value
        """
        return self.forward_actor(features), self.forward_critic(features)

    def forward_actor(self, policy_input: th.Tensor) -> th.Tensor:
        policy_output = self.policy_net(policy_input)
        return policy_output

    def forward_critic(self, value_input: th.Tensor) -> th.Tensor:
        value_output = self.value_net(value_input)
        return value_output

class CustomActorCriticPolicy(ActorCriticPolicy):
    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Callable[[float], float],
        *args,
        **kwargs,
    ):
        # 正交初始化
        kwargs["ortho_init"] = False
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            # 将其余参数传递给基类
            *args,
            **kwargs,
        )
    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = CustomNetwork(self.features_dim)

def linear_schedule(initial_value: float, lowest_value: float = 0.000) -> Callable[[float], float]:
    def func(progress_remaining: float) -> float:  # 计算当前学习率
        return progress_remaining * initial_value + lowest_value
    return func

c_range = 0.25  # 表示奖励函数中的一个参数，用于控制奖励函数的范围
ent_coef = 0.0016  # 表示策略的熵系数，用于控制策略的探索性
learning_rate = 0.0003 # 0.0003
_clip_range = linear_schedule(c_range)
_learning_rate = linear_schedule(learning_rate, lowest_value=0.0001)
running_time = datetime.now().strftime("%m%d%H%M%S")

callback = CallbackList([])
root_dir_tensorboard = '/home/wfh/PiH-RL/tensorboard_0418/'
root_dir_model ="/home/wfh/桌面/TX_PiH_R/PiH_model/"
recording_path = '/home/wfh/PiH-RL/recording1215/' + running_time

env = gym.make('tx2USB-v1',render=True)
algorithm = 'PPO_network'
if algorithm == 'PPO_network':
    model = PPO(CustomActorCriticPolicy,
                env=env,
                #device='cuda:0',
                verbose=1,
                ent_coef=ent_coef,
                clip_range=_clip_range,  # 策略梯度的剪切范围
                learning_rate=_learning_rate,
                n_steps=2048,
                batch_size=64,
                n_epochs=10,
                tensorboard_log=root_dir_tensorboard)  # tesorboard日志地址
elif algorithm == 'TD3':
    model = TD3(
        policy="MultiInputPolicy",  # 策略网络的类型
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
model.learn(total_timesteps=int(1e5), callback=callback, tb_log_name="txPiH_run_"+ algorithm +'_'+running_time)  # (训练steps)
model.save(root_dir_model+running_time+".pkl")  # 保存
#model = PPO.load(root_dir_model, env=None)
#mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10, deterministic=True, render=False)
#print(f"Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")
env.close()