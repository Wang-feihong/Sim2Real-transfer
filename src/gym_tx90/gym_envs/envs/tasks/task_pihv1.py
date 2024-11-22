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
import os
import pandas as pd
from datetime import datetime

class PeginHole(Task):
    def __init__(self,sim, _normalize)-> None:
        self.r_step = 0
        # self.goal_range_low = np.array([-0.372, -0.045, 1.005]) #([-0.3745, -0.0485, 1.005])  # 目标随机范围 孔
        # self.goal_range_high = np.array([-0.377, -0.050, 1.006]) #([-0.3755, -0.495, 1.006])
        self.goal_range_low = np.array([-0.374, -0.048, 1.005]) #([-0.3745, -0.0485, 1.005])  # 目标随机范围 孔
        self.goal_range_high = np.array([-0.376, -0.050, 1.0055]) #([-0.3755, -0.495, 1.006])
        norm_max = 1
        norm_min = -1
        self.hole_size = self.goal_range_high - self.goal_range_low
        self.norm_size = (norm_max - norm_min) / self.hole_size
        self.hole_mean = (self.goal_range_high + self.goal_range_low) / 2
        self.norm_mean = (norm_max + norm_min) / 2 * np.array([1, 1, 1])
        self._normalize_obs = _normalize
        super().__init__(sim)

    # 获取观测值
    def get_obs(self) ->np.ndarray:
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
        self.goal = self._sample_goal()
        # self.goal = np.array([0.0, 0.36, 0.91])
        desired_goal = np.copy(self.goal)
        self.sim.set_mocap_pos(mocap="box", pos=desired_goal)
        # print("mocap_box:",self.sim.set_mocap_pos(mocap="box", pos=desired_goal))
        x_deg = (2.0 * np.random.random() + (-1.0)) * 5
        y_deg = (2.0 * np.random.random() + (-1.0)) * 5
        z_deg = (2.0 * np.random.random() + (-1.0)) * 5  # random---[0,1）
        # # desired_quat = euler_to_quaternion(z_deg * np.pi/180, (-90+xy_deg) * np.pi/180, 0)
        # desired_quat = euler_to_quaternion(xy_deg * np.pi/180, 0, z_deg * np.pi/180)  # rxz2
        # desired_quat = euler_to_quaternion((-0+xy_deg) * np.pi/180, 0, z_deg * np.pi/180)
        desired_quat = euler_to_quaternion((-0+x_deg) * np.pi/180, (-0+y_deg) * np.pi/180, z_deg * np.pi/180)
        self.sim.set_mocap_quat(mocap="box", quat=desired_quat)

    def reset_ (self) -> None:
        self.goal = self._sample_goal()
        # self.goal = np.array([0.0, 0.36, 0.91])
        desired_goal = np.copy(self.goal)
        self.sim.set_mocap_pos(mocap="box", pos=desired_goal)
        # print("mocap_box:",self.sim.set_mocap_pos(mocap="box", pos=desired_goal))
        # 训练
        a = 2
        # z_deg = (2.0 * np.random.random() + (-1.0)) * a  # random---[0,1）0728
        # x_deg = (2.0 * np.random.random() + (-1.0)) * a
        # y_deg = (2.0 * np.random.random() + (-1.0)) * a
        # 测试
        z_deg = (a + np.random.random()) * np.random.choice([-1, 1])
        x_deg = (a + np.random.random()) * np.random.choice([-1, 1])
        y_deg = (a + np.random.random()) * np.random.choice([-1, 1])
        # # desired_quat = euler_to_quaternion(z_deg * np.pi/180, (-90+xy_deg) * np.pi/180, 0)
        # desired_quat = euler_to_quaternion(xy_deg * np.pi/180, 0, z_deg * np.pi/180)  # rxz2
        # desired_quat = euler_to_quaternion((-0+xy_deg) * np.pi/180, 0, z_deg * np.pi/180)
        desired_quat = euler_to_quaternion((-0+x_deg) * np.pi/180, (-0+y_deg) * np.pi/180, z_deg * np.pi/180)
        self.sim.set_mocap_quat(mocap="box", quat=desired_quat)
        self.save_data_to_excel(desired_goal, z_deg, x_deg, y_deg)

    def save_data_to_excel(self, desired_goal, z_deg, x_deg, y_deg):
        self.file_counter += 1
        # 创建DataFrame
        self.hole_angle.append({
            'desired_goal_x': round(desired_goal[0], 5),
            'desired_goal_y': round(desired_goal[1], 5),
            'desired_goal_z': round(desired_goal[2], 5),
            'x_deg': round(x_deg, 5),
            'y_deg': round(y_deg, 5),
            'z_deg': round(z_deg, 5),
        })
        if self.file_counter % 100 == 0:
            df = pd.DataFrame(self.hole_angle)
            # file_name = f"{self.file_counter}.xlsx"
            #file_name = f"hole_angle{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
            file_name = "hole_angle4-0828_six01v1.xlsx"
            file_path = os.path.join("/home/wfh1/PiH-RL/randomhole/six-angle", file_name)
            df.to_excel(file_path, index=False)

    def _sample_goal(self) -> np.ndarray:
        """目标随机化."""
        goal = np.random.uniform(self.goal_range_low, self.goal_range_high)
        return goal

    def get_achieved_goal(self) -> np.ndarray:
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
