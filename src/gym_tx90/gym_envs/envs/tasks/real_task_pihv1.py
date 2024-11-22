'''
Name: 
Date: 2024-04-18 14:53:14
Creator: 王飞鸿
Description: 
'''
from ctypes.wintypes import PINT
from typing import Any, Dict, Union
import math
import numpy as np
from gym_envs.envs.core2usb import Task
from gym_envs.utils import distance, normalizeVector, euler_to_quaternion
from threading import Thread
import time

class PeginHole(Task):
    def __init__(self,sim, _normalize,real_robot: bool = False,)-> None:
        self.r_step = 0
        # self.goal_range_low = np.array([-0.372, -0.045, 1.005]) #([-0.3745, -0.0485, 1.005])  # 目标随机范围 孔
        # self.goal_range_high = np.array([-0.377, -0.050, 1.006]) #([-0.3755, -0.495, 1.006])
        # self.goal_range_low = np.array([-0.374, -0.048, 1.005]) #([-0.3745, -0.0485, 1.005])  # 目标随机范围 孔  0601
        # self.goal_range_high = np.array([-0.376, -0.050, 1.0055]) #([-0.3755, -0.495, 1.006])
        # self.goal_range_low = np.array([0.474, 0.049, 1.003+0.0995]) #([-0.3745, -0.0485, 1.005])  # 0703之前目标随机范围 孔  0601根据实机改后
        # self.goal_range_high = np.array([0.476, 0.051, 1.10035+0.0995]) #([-0.3755, -0.495, 1.006])
        self.goal_range_low = np.array([0.474, -0.051, -1.0035-0.1035]) #([-0.3745, -0.0485, 1.005])  # 0703改目标随机范围 孔  0601根据实机改后
        self.goal_range_high = np.array([0.476, -0.049, -1.003-0.1035]) #([-0.3755, -0.495, 1.006])
        norm_max = 1
        norm_min = -1
        self.hole_size = self.goal_range_high - self.goal_range_low
        self.norm_size = (norm_max - norm_min) / self.hole_size
        self.hole_mean = (self.goal_range_high + self.goal_range_low) / 2
        self.norm_mean = (norm_max + norm_min) / 2 * np.array([1, 1, 1])
        self._normalize_obs = _normalize
        self.real_robot = real_robot
        super().__init__(sim)

    # 获取观测值
    def get_obs(self) ->np.ndarray:
        if self.real_robot == True:
            # hole_x_offset = -0.375 + (2.0 * np.random.random() + (-1.0)) * 0.001  # 0601根据实机改前
            # hole_y_offset = -0.049 + (2.0 * np.random.random() + (-1.0)) * 0.001
            # hole_z_offset = 1.005 + (2.0 * np.random.random() + (-1.0)) * 0.0005
            # hole_x_offset = 0.475 + (2.0 * np.random.random() + (-1.0)) * 0.001  # 0601根据实机改后
            # hole_y_offset = 0.050 + (2.0 * np.random.random() + (-1.0)) * 0.001
            # hole_z_offset = 1.003 + (2.0 * np.random.random() + (-1.0)) * 0.0005
            # hole_position = np.array([hole_x_offset,
            #                           hole_y_offset,
            #                           hole_z_offset])
            # hole_position[0] += (2.0 * np.random.random() + (-1.0)) * 0.003
            # hole_position[1] += (2.0 * np.random.random() + (-1.0)) * 0.003
            # hole_position[2] += (2.0 * np.random.random() + (-1.0)) * 0.001
            hole_position=self.reset()
            # print("hole_pos:", hole_position)
            if self._normalize_obs is True:
                hole_position = (hole_position - self.hole_mean) * self.norm_size + self.norm_mean
            obs = np.copy(hole_position)
            return obs
        else:
            # hole_top_p = np.copy(self.sim.get_site_position("hole_top"))
            hole_bot_p = np.copy(self.sim.get_site_position("hole_bottom"))
            # # 归一化处理0330
            # def normalize(data, min_val, max_val):
            #     return (data - min_val) / (max_val - min_val)
            # #hole_top_p = normalize(hole_top_p, self.goal_range_low, self.goal_range_high)
            if self._normalize_obs is True:
                hole_bot_p = (hole_bot_p - self.hole_mean) * self.norm_size + self.norm_mean
            obs = np.copy(hole_bot_p)
            # obs = []
            return obs
    # 重置
    def reset(self) -> None:
        if self.real_robot == True:
            self.goal = self._sample_goal()
            desired_goal = np.copy(self.goal)
            desired_goal = np.array([0.475+0.0005,-0.05-0.0005,-1.1065-0.0001])  # 偏差1
            # desired_goal = np.array([0.475-0.001,-0.05+0.0000,-1.1065-0.0000])  # 偏差1
            time.sleep(0.01)
            # hole_x = -0.375
            # hole_y = -0.049
            # hole_z = 1.005
            # hole_x = 0.475  # 0601根据实机改后
            # hole_y = 0.050
            # hole_z = 1.003
            # hole_position = np.array([hole_x,
            #                 hole_y,
            #                 hole_z])

            hole_x_offset = 0.475 + (2.0 * np.random.random() + (-1.0)) * 0.001  # 0605根据实机改后
            # hole_y_offset = 0.050 + (2.0 * np.random.random() + (-1.0)) * 0.001
            # hole_z_offset = 1.003 + (2.0 * np.random.random() + (-1.0)) * 0.0005 + 0.0915 + 0.008 #0.012改为0.010，尽快达到成功奖励
            hole_y_offset = -0.050 - (2.0 * np.random.random() + (-1.0)) * 0.001   # 0703设置
            hole_z_offset = -1.003 - (2.0 * np.random.random() + (-1.0)) * 0.0005 - 0.0915 - 0.012 #0.012改为0.010，尽快达到成功奖励
            hole_position = np.array([hole_x_offset,
                                      hole_y_offset,
                                      hole_z_offset])
            # return hole_position
            return desired_goal
        else:
            self.goal = self._sample_goal()
            # self.goal = np.array([0.0, 0.36, 0.91])
            desired_goal = np.copy(self.goal)
            self.sim.set_mocap_pos(mocap="box", pos=desired_goal)
            # print("mocap_box:",self.sim.set_mocap_pos(mocap="box", pos=desired_goal))
            z_deg = (2.0 * np.random.random() + (-1.0)) * 5  # random---[0,1）
            xy_deg = (2.0 * np.random.random() + (-1.0)) * 5
            # # desired_quat = euler_to_quaternion(z_deg * np.pi/180, (-90+xy_deg) * np.pi/180, 0)
            # desired_quat = euler_to_quaternion(xy_deg * np.pi/180, 0, z_deg * np.pi/180)  # rxz2
            # desired_quat = euler_to_quaternion((-0+xy_deg) * np.pi/180, 0, z_deg * np.pi/180)
            desired_quat = euler_to_quaternion((-0+xy_deg) * np.pi/180, (-0+xy_deg) * np.pi/180, z_deg * np.pi/180)
            self.sim.set_mocap_quat(mocap="box", quat=desired_quat)

    def _sample_goal(self) -> np.ndarray:
        """目标随机化."""
        goal = np.random.uniform(self.goal_range_low, self.goal_range_high)
        return goal

    def get_achieved_goal(self) -> np.ndarray:
        if self.real_robot == True:
            object_position = self.get_obs()  # 用于计算奖励而已
        else:
            object_position = np.copy(self.sim.get_site_position("usb_bottom"))
        return object_position

    # def is_success(self, achieved_goal: np.ndarray, desired_goal: np.ndarray) -> Union[np.ndarray, float]:
    def is_success(self, achieved_goal: np.ndarray, desired_goal: np.ndarray):
        target_bottom = np.copy(self.sim.get_site_position("hole_bottom"))
        d_bottom = distance(achieved_goal, target_bottom)
        r_bot = 1 - math.tanh(100 * d_bottom)
        d = r_bot
        return np.array(d > 0.875, dtype=np.float64)
        # return d > 0.875

    # 奖励计算
    def compute_reward(self, achieved_goal, desired_goal, info: Dict[str, Any]) -> Union[np.ndarray, float]:
        # 目标位置和当前状态的信息
        target_bottom = np.copy(self.sim.get_site_position("hole_bottom"))
        target_top = np.copy(self.sim.get_site_position("hole_top"))
        usb_top = np.copy(self.sim.get_site_position("ubs_top"))
        usb_bottom = np.copy(self.sim.get_site_position("ubs_bottom"))
        # ------------------------
        # 对步数比例值进行tanh计算，将其限制在0.3到1之间，并根据 step_gain 进行调整

        # 步数惩罚---走的步数越多 惩罚越大
        # self.r_step -= 0.0005
        self.r_step += 1
        # print(self.r_step)
        total_steps = 350
        step_ratio = math.tanh((self.r_step / total_steps) * 10) + 0.3
        _step_val = - self.r_step / total_steps  # 一个回合350步
        # step_ratio = math.tanh(self.r_step / 100)
        step_val = _step_val * step_ratio
        # print(step_val)

        # 步数惩罚--tanh归一化 走的步数越多 惩罚越大
        if self.r_step >= total_steps-20:
            step_gain = (math.tanh((total_steps - self.r_step) / 10) - 1) / 20
        else:
            step_gain = 0

        # 距离奖励---三类不同权重的奖励值
        # 定义权重因子，进行奖励缩放
        scaled_ratio_top = np.array([2, 2, 1]) #np.array([1.25, 1.25, 1])
        scaled_ratio_mid = np.array([1, 1, 1])
        scaled_ratio_bot = np.array([1, 1, 2]) #np.array([1, 1, 1.25])
        d_bottom = distance(achieved_goal * scaled_ratio_bot, target_bottom * scaled_ratio_bot)
        d_center = distance(achieved_goal * scaled_ratio_mid, desired_goal * scaled_ratio_mid)
        d_top = distance(achieved_goal * scaled_ratio_top, target_top * scaled_ratio_top)
        r_top = (1 - math.tanh(50 * d_top)) * 0.2
        r_center = (1 - math.tanh(30 * d_center)) * 0.3
        r_bot = (1 - math.tanh(10 * d_bottom))

        _r_top = -d_top * 0.1
        _r_center = -d_center * 0.15
        _r_bot = -d_bottom * 1.5
        _r = (_r_top + _r_center + _r_bot)

        # 成功奖励/失败惩罚---# 判断是否成功插入目标孔
        if r_bot > 0.865:
            # 增加一个成功率的系数
            self.suc_ratio += 0.1
            get_suc = 2.5 * self.suc_ratio
        else:
            self.suc_ratio = 1
            get_suc = 0
        scaled_ratio_toptop = np.array([8, 8, 0.1])  # xyz权重 xy平面距离影响大

        # 距离奖励---轴孔距离奖励 距离越小奖励越大 （0-0.1）
        d_objtop_holetop = distance(usb_top * scaled_ratio_toptop, target_top * scaled_ratio_toptop)
        r_objtop_holetop = (1 - math.tanh(20 * d_objtop_holetop)) * 0.1

        # 步数惩罚---根据剩余步数、阈值步数来计算惩罚值 剩余步数越低于阈值，惩罚越高 剩余步数大于阈值无惩罚（-0.995~0）
        left_step = total_steps - self.r_step  #总步数350
        threashold_step = 200
        if left_step < threashold_step:
            step_v = (left_step-threashold_step) / (threashold_step * 1)
        else:
            step_v = 0

        return (_r_top + _r_center + _r_bot) + get_suc + step_gain

