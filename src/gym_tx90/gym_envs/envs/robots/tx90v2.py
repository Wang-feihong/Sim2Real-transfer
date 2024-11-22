'''
Name: 
Date: 2024-04-18 14:53:33
Creator: 王飞鸿
Description: 
'''
from array import array
from email.mime import base
from typing import Dict, Optional
import numpy as np
import math
from gym import spaces
from gym_envs.envs.core2usb import MujocoRobot
from datetime import datetime
from ctypes import util
import time
import numpy as np
from gym_envs.utils import distance, normalizeVector, euler_to_quaternion
from scipy.spatial.transform import Rotation as R
import math

class TX_90(MujocoRobot):
    def __init__(self, sim,_normalize,) -> None:
        # n_action = 6
        n_action = 12
        pxzy = 0.0002
        rxzy = 0.1 #0.05
        # action_space = spaces.Box(-1, 1, shape=(n_action,), dtype=np.float32) # 连续动作空间
        action_space = spaces.Discrete(n_action) # 离散动作空间
        # action_space = spaces.Tuple([spaces.Discrete(n_action, start=0),
        #                             spaces.Discrete(n_action, start=0),
        #                             spaces.Discrete(n_action, start=0),
        #                             spaces.Discrete(n_action, start=0),
        #                             spaces.Discrete(n_action, start=0),
        #                             spaces.Discrete(n_action, start=0)])
        # 使用Tuple组合六个Discrete空间  
        # action_space = spaces.Tuple([  
        #     spaces.Discrete(n_action),  
        #     spaces.Discrete(n_action),  
        #     spaces.Discrete(n_action),  
        #     spaces.Discrete(n_action),  
        #     spaces.Discrete(n_action),  
        #     spaces.Discrete(n_action)  
        # ])
        self._action_to_direction = {
            0: np.array([pxzy,0,0]),
            1: np.array([-pxzy,0,0]),
            2: np.array([0,pxzy,0]),
            3: np.array([0,-pxzy,0]),
            4: np.array([0,0,pxzy]),
            5: np.array([0,0,-pxzy]), #xyz位移
            6: np.array([rxzy,0,0]),
            7: np.array([-rxzy,0,0]),
            8: np.array([0,rxzy,0]),
            9: np.array([0,-rxzy,0]),
            10: np.array([0,0,rxzy]),
            11: np.array([0,0,-rxzy]), #xyz旋转
        }
        print("action_space",action_space)
        # self.force_threshold = True
        self.dense = False
        self.ee_pos_low = np.array([-0.39, 0.04, 0.95]) #np.array([-0.4, 0.2, 0.5])  # 旋转90度 6轴末端 [ 0.04969808 -0.37503659  1.07749758]
        self.ee_pos_high = np.array([-0.36,  0.06, 1.5]) #np.array([0,  0.45, 1.2])  #0.36-0.39 0.04-0.06
        # self.ee_pos_low = np.array([-1, -1, 0.95]) #np.array([-0.4, 0.2, 0.5])  # 旋转90度 6轴末端 [ 0.04969808 -0.37503659  1.07749758]
        # self.ee_pos_high = np.array([1,  1, 1.5]) #np.array([0,  0.45, 1.2]) #0429关
        self.ee_rot_low = np.array([-10, -10, -180])
        self.ee_rot_high = np.array([10, 10, 180])
        ft_xyz = 15  # 0601将20牛改为10牛
        ft_rxyz = 0.5  # # 0601将2牛米改为0.15
        self.ft_sensor_low = np.array([-ft_xyz, -ft_xyz, -ft_xyz, -ft_rxyz, -ft_rxyz, -ft_rxyz])
        self.ft_sensor_high = np.array([ft_xyz, ft_xyz, ft_xyz, ft_rxyz, ft_rxyz, ft_rxyz])
        self.ee_dis_ratio = 1 #0.00085
        self.ee_dis_ratioxy = 1 #0.00085
        self.ee_dis_ratioz = 1 #0.00085
        self.ee_rotxy_ratio = 0
        self.ee_rotz_ratio = 0.0001
        self._dis_ratio = 0.0001
        self._rot_ratio = 0.01
        self.planner_time = 1
        self.low_filter_gain = 0.1
        self.ft_f_max = 20
        self.ft_t_max = 2
        norm_max = 1
        norm_min = -1
        # ee pos norm 参数 --------
        self.ee_pos_scale = self.ee_pos_high - self.ee_pos_low
        self.ee_pos_norm_scale = (norm_max - norm_min) / self.ee_pos_scale
        self.ee_pos_mean = (self.ee_pos_high + self.ee_pos_low) / 2
        self.ee_pos_norm_mean = (norm_max + norm_min) / 2 * np.array([1, 1, 1])
        # ee rot norm 参数 --------
        self.ee_rot_scale = self.ee_rot_high - self.ee_rot_low
        self.ee_rot_norm_scale = (norm_max - norm_min) / self.ee_rot_scale
        self.ee_rot_mean = (self.ee_rot_high + self.ee_rot_low) / 2
        self.ee_rot_norm_mean = (norm_max + norm_min) / 2 * np.array([1, 1, 1])
        # F/T sensor norm 参数 ---------
        self.ft_scale = self.ft_sensor_high - self.ft_sensor_low
        self.ft_norm_scale = (norm_max - norm_min) / self.ft_scale
        self.ft_mean = (self.ft_sensor_high + self.ft_sensor_low) / 2
        self.ft_norm_mean = (norm_max + norm_min) / 2 * np.array([1, 1, 1, 1, 1, 1])
        self.ft_last = np.array([0, 0, 0, 0, 0, 0])
        self.ft_last_obs = np.array([0, 0, 0, 0, 0, 0])
        self._normalize_obs =_normalize
        super().__init__(
            sim,
            action_space=action_space,
            joint_index=np.array([0, 1, 2, 3, 4, 5, 6]),
            #joint_forces=np.array([87.0, 87.0, 87.0, 87.0, 12.0, 120.0, 170.0]),
            joint_list=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
            sensor_list=["touchsensor_r1", "touchsensor_r2", "touchsensor_r3", "touchsensor_r4", "touchsensor_r5", "touchsensor_r6",
                         "touchsensor_l1", "touchsensor_l2", "touchsensor_l3", "touchsensor_l4", "touchsensor_l5", "touchsensor_l6"]
            )
        #self.init_joint = np.array([0.754, 0.66, -1.54, 0, -0.88, 0]) #面上
        #self.init_joint = np.array([0.723, 0.314, -1.01, 0, -0.723, 0]) #上方
        #self.init_joint = np.array([1.57, 0., 0., 0., 0., 0]) # 正面 旋转90
        self.init_joint = np.array([0., 0., 0., 0., 0., 0.])
    def index_to_action(index):  
        # 将整数索引转换为基数为n_sub_actions的数字的数组  
        digits = np.unravel_index(index, 3 * 6)  
        # digits现在是一个元组，其中每个元素是一个从0到n_sub_actions-1的整数  
        # 你可以根据需要重新排列或解释这些数字来代表具体的动作组合  
        # 例如，假设digits[0:3]是移动，digits[3:6]是转动  
        moves = digits[:3]  
        rotates = digits[3:]  
        return moves, rotates  

    # 动作设计
    def set_action(self, action:np.ndarray) -> None:
        _action = self._action_to_direction[action]

        # action = self.index_to_action(action)
        # action = action.copy()
        # ee_displacement = np.array(list(action))
        # print("动作空间1：",action)
        # action = np.clip(action, self.action_space.low, self.action_space.high) # for dense
        displacement = np.copy(_action)
        # print("displacement1:",displacement)
        if 0 <= action <= 5:
            ee_displacement = np.concatenate((displacement, np.zeros(3)))
        else:
            ee_displacement = np.concatenate((np.zeros(3), displacement))
        # print("displacement2：",ee_displacement)
        target_arm_angles = self.ee_displacement_to_target_arm_angles(ee_displacement)
        # print("target_arm_angles：",target_arm_angles)
        current_joint_arm = np.array([self.get_joint_angle(joint=self.joint_list[i]) for i in range(6)])
        if np.isnan(target_arm_angles).any() is True or np.isfinite(target_arm_angles).all() is False:
            target_arm_angles = np.copy(current_joint_arm)
        # current_joint_arm = np.array([self.get_joint_angle(joint=self.joint_list[i]) for i in range(6)])
        # print("target_arm_angles:",target_arm_angles)
        # print("current_joint_arm:",current_joint_arm)

        test_pos_rot = (self.sim.forward_kinematics(target_arm_angles))
        
        # print("test_pos",test_pos_rot[0],"test_rot:",R.from_quat(test_pos_rot[1]).as_euler("xyz",degrees=True))
        self.joint_planner(_time=self.planner_time/50,
                    current_joint=current_joint_arm,
                    target_joint=target_arm_angles)
    # 获取观测值
    def get_obs(self) -> float:
        ee_pos = self.sim.get_site_position("attachment_site")
        ee_rot = R.from_matrix(self.sim.get_site_mat("attachment_site").reshape(3,3)).as_euler('xyz', degrees=True)
        #print("位置：",ee_pos)
        #print("姿态：",ee_rot)
        #print('quat:',self.sim.get_body_quaternion("wrist_3_link"))
        ft_sensor = self.sim.get_ft_sensor(force_site="ee_force_sensor", torque_site="ee_torque_sensor").copy()
        #print("ft_sensor:", ft_sensor)
        ft_sensor = self.ft_last_obs * self.low_filter_gain + ft_sensor * (1 - self.low_filter_gain)
        self.ft_last_obs = np.copy(ft_sensor)
        current_joint = np.array([self.get_joint_angle(joint=self.joint_list[i]) for i in range(6)])
        # 定义归一化处理函数
        # def normalize(data, min_val, max_val):
        #     return (data - min_val) / (max_val - min_val)
        #ee_pos = normalize(ee_pos, self.ee_pos_low, self.ee_pos_high)
        #ee_rot = normalize(ee_rot, self.ee_rot_low, self.ee_rot_high)
        ee_pos = np.copy(ee_pos)
        ee_rot = np.copy(ee_rot)
        #ft_sensor = normalize(ft_sensor, self.ft_sensor_low, self.ft_sensor_high)
        #print("归一化后ft_sensor:", ft_sensor)
        #self.ft_last_obs = np.copy(ft_sensor)
        if self._normalize_obs is True:
            ee_pos = (ee_pos - self.ee_pos_mean) * self.ee_pos_norm_scale + self.ee_pos_norm_mean
            ee_rot = (ee_rot - self.ee_rot_mean) * self.ee_rot_norm_scale + self.ee_rot_norm_mean
            ft_sensor = (ft_sensor - self.ft_mean) * self.ft_norm_scale + self.ft_norm_mean
        # obs = np.concatenate((ee_pos, ee_rot, ft_sensor))
        obs = np.concatenate((ee_pos, ee_rot, ft_sensor,current_joint))
        #print('机器人状态空间obs:',obs)
        # obs = ee_pos
        return obs
    def get_obs_dict(self) -> Dict[str, np.ndarray]:
        ee_pos = self.sim.get_site_position("attachment_site")
        ee_rot = R.from_matrix(self.sim.get_site_mat("attachment_site").reshape(3, 3)).as_euler('xyz', degrees=True)
        ft_sensor = self.sim.get_ft_sensor(force_site="ee_force_sensor", torque_site="ee_torque_sensor")
        # 创建一个字典用于存储观测空间的各个部分
        obs = {
            'ee_pos': np.copy(ee_pos),
            'ee_rot': np.copy(ee_rot),
            'ft_sensor': np.copy(ft_sensor)
        }
        return obs
    # 重置
    def reset(self) -> None:
        self.sim.reset()
        self.control_joints_init()
        self.init_ft_sensor = self.sim.get_ft_sensor(force_site="ee_force_sensor", torque_site="ee_torque_sensor")
    # 关节角更新调整 齐次变换
    def ee_displacement_to_target_arm_angles(self, ee_displacement: np.ndarray) ->np.ndarray:
        if self.dense==True:
            ee_displacement[:3] = ee_displacement[:3] * self.ee_dis_ratio
            ee_displacement[3:-1] = ee_displacement[3:-1] * self.ee_rotxy_ratio
            ee_displacement[-1] = ee_displacement[-1] * self.ee_rotz_ratio
        else:
            # ee_displacement = [a + (a - 1) * self._dis_ratio for a in ee_displacement]
            # ee_displacement = [a + (a - 1) * self._rot_ratio for a in ee_displacement]
            # ee_displacement[:3] = [(x - 1) * self._dis_ratio for x in ee_displacement[:3]] # 6个离散动作
            # ee_displacement[3:] = [(x - 1) * self._rot_ratio for x in ee_displacement[3:]]
            ee_displacement = ee_displacement
            # ee_displacement[0] = ee_displacement[0] + [ee_displacement-1]* self.ee_dis_ratioxy
            # ee_displacement[:2] = ee_displacement[:2] * self.ee_dis_ratioxy
            # ee_displacement[2] = ee_displacement[2] * self.ee_dis_ratioz
            # ee_displacement[3:-1] = ee_displacement[3:-1] * self.ee_rotxy_ratio
            # ee_displacement[-1] = ee_displacement[-1] * self.ee_rotz_ratio
            # 为加快收敛，动作数3
            # ee_displacement[:2] = ee_displacement[:2] * self.ee_dis_ratioxy
            # ee_displacement[2] = ee_displacement[2] * self.ee_dis_ratioz
            # ee_displacement[3:-1] = ee_displacement[3:-1] * self.ee_rotxy_ratio
            # ee_displacement[-1] = ee_displacement[-1] * self.ee_rotz_ratio
            # print("ee_displacement2:",ee_displacement)
            # ee_displacement = [0.005,0,0,0,0,0]
            # current_joint = np.array([self.get_joint_angle(joint=self.joint_list[i]) for i in range(6)])
        '''
        current_ee_pos = self.sim.get_body_position('attachment_site')
        current_ee_rot = self.sim.get_body_quaternion("attachment_site")
        print("wrist_3_link:",current_ee_pos)
        print('usb_bottom:',self.sim.get_site_position("usb_bottom"))
        print('hole_bottom:',self.sim.get_site_position("hole_bottom"))
        rotation_matrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # 旋转矩阵
        translation_matrix = np.array([0, 0, 2.08]).reshape(3, 1)  # 位移矩阵
        # 构建齐次变换矩阵
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = rotation_matrix
        homogeneous_matrix[:3, 3] = translation_matrix.flatten()
        # 执行坐标系变换
        current_ee_pos = homogeneous_matrix.dot(np.append(current_ee_pos, 1))[:3]
        print("current_ee_pos:",current_ee_pos)
        current_ee_rot = R.from_euler('x', 180, degrees=True).apply(R.from_quat(current_ee_rot).as_euler("xyz", degrees=True))
        '''
        current_joint = np.array([self.get_joint_angle(joint=self.joint_list[i]) for i in range(6)])
        #print("初始关节角：",current_joint)
        current_ee_pos = (self.sim.forward_kinematics(current_joint))[0]
        current_ee_qua = np.array((self.sim.forward_kinematics(current_joint))[1])
        current_ee_eul = R.from_quat(current_ee_qua).as_euler("xyz",degrees=True)
        # print("current_pos:",current_ee_pos,"current_qua:",current_ee_qua,"current_eul:",current_ee_eul)
        # print("current_pos:",current_ee_pos,"current_eul:",current_ee_eul)
        # ee_displacement = [0.,0,0.0001,0,0,0]
        target_ee_pos = current_ee_pos + ee_displacement[:3]
        target_ee_eul = current_ee_eul + ee_displacement[3:]
        # print("target_pos1:",target_ee_pos,"target_eul1:",target_ee_eul)
        target_ee_rot = np.clip(target_ee_eul, self.ee_rot_low, self.ee_rot_high)
        target_ee_pos = np.clip(target_ee_pos, self.ee_pos_low, self.ee_pos_high)
        # target_ee_pos = current_ee_pos + [0.0, 0., 0.]
        # target_ee_rot = current_ee_eul + [0., 0 , 0]
        # print("target_ee_pos2:",target_ee_pos,"target_ee_rot2:",target_ee_rot)
        # 为加快收敛，quat设为0,0,1,0
        target_ee_quat = R.from_euler('xyz', target_ee_rot, degrees=True).as_quat()
        # target_ee_quat = [0,0,1,0]
        target_arm_angles = self.inverse_kinematics(
        current_joint=current_joint,
        target_position=target_ee_pos,
        target_orientation=target_ee_quat)
        time.sleep(0.1)
        return target_arm_angles

    # 控制关节运动
    def control_joints_init(self) ->None:
        self.set_joint_angles(self.init_joint)
        self.sim.step()
        self.control_joints(self.init_joint)

    # 关节角更新次数、运动
    def joint_planner(self, _time: float, current_joint: np.ndarray, target_joint: np.ndarray) -> None:
        delta_joint = target_joint - current_joint
        planner_steps = int(_time / self.sim.timestep)
        for i in range(planner_steps):
            planned_delta_joint = delta_joint * math.sin((math.pi/2)*(i/planner_steps))
            target_joint = planned_delta_joint + current_joint
            self.control_joints(target_joint)
            self.sim.step()

