'''
Name: 
Date: 2024-04-18 14:53:33
Creator: 王飞鸿
Description: 
'''
from abc import ABC, abstractmethod
import math
from typing import Any, Dict, Optional, Tuple, Union
from gym import spaces
import gym.utils.seeding
import gym_robotics
import numpy as np
from gym_envs.utils import distance, normalizeVector, euler_to_quaternion
import pandas as pd
# 机器人功能设置 动作空间
class MujocoRobot(ABC):
    def __init__(
        self,
        sim,
        action_space: gym.spaces.Space,
        joint_index: np.ndarray,
        #joint_forces: np.ndarray,
        joint_list: list,
        sensor_list: list,
    ) -> None:
        self.sim = sim
        self.setup()  # 调用setup()方法，该方法在子类中可以被重写以进行一些初始化操作
        self.action_space = action_space
        print("act space:",self.action_space)
        self.joint_index = joint_index
        #self.joint_forces = joint_forces
        self.joint_list = joint_list
        self.sensor_list = sensor_list
        ft_xyz = 60
        ft_rxyz = 20
        # self.f_sensor_thresh = np.array([ft_xyz, ft_xyz, ft_xyz])
        self.f_sensor_thresh = np.array([20, 20, 20])
        # self.t_sensor_thresh = np.array([ft_rxyz, ft_rxyz, ft_rxyz])
        self.t_sensor_thresh = np.array([2, 2, 2])  # 0531实验 因为单位问题 力矩数据扩大1000倍

    # @abstractmethod
    # def reset(self) -> None:
    #     """Reset the robot and return the observation."""
    @abstractmethod
    def set_action(self, action: np.ndarray) -> None:
        """Set the action. Must be called just before sim.step().
        设置机器人的动作。
        该方法在调用sim.step()之前必须被调用。子类需要实现这个方法来定义具体的动作设置逻辑。
        Args:
            action (np.ndarray): The action.
        """
    @abstractmethod
    def get_obs(self) -> np.ndarray:
        """Return the observation associated to the robot.
        获取与机器人相关的观测
        Returns:
            np.ndarray: The observation.
        """
    @abstractmethod
    def reset(self) -> None:
        """Reset the robot and return the observation."""

    def setup(self) -> None:
        """Called after robot loading.在机器人加载后进行一些设置操作。它没有参数，也没有返回值。"""
        pass

    @abstractmethod
    def get_obj_pos(self) -> np.ndarray:
        """
        get obj_pos
        """
    @abstractmethod
    def get_real_ft_sensor(self) -> np.ndarray:
        """
        get get_real_ft_sensor
        """
    def set_forwad(self) -> None:
        self.sim.set_forward()
    # 获取位置
    def get_body_position(self, body: str) -> np.ndarray:
        return self.sim.get_body_position(body=body)
    # 获取速度
    def get_body_velocity(self, body: str) -> np.ndarray:
        return self.sim.get_body_velocity(body=body)
    # 获取传感器信息
    def get_touch_sensor(self, sensor: str) -> np.ndarray:
        return self.sim.get_touch_sensor(sensor=sensor)
    def get_ft_sensor(self, force_site: str, torque_site: str) -> np.ndarray:
        return self.sim.get_touch_sensor(force_site=force_site, torque_site=torque_site)
    # 获取关节角
    def get_joint_angle(self, joint: str) -> float:
        return self.sim.get_joint_angle(joint=joint)
    # 获取关节速度
    def get_joint_velocity(self, joint: str) -> float:
        return self.sim.get_joint_velocity(joint=joint)
    # 控制关节运动
    def control_joints(self, target_angles: np.ndarray) -> None:
        self.sim.control_joints(target_angles=target_angles)
    # 给定关节角，传参
    def set_joint_angles(self, angles: np.ndarray) -> None:
        self.sim.set_joint_angles(angles=angles)
    # 逆运动学计算 当前的估计关节角度，目标位置，目标姿态
    def inverse_kinematics(self, current_joint: np.ndarray, target_position: np.ndarray, target_orientation: np.ndarray) -> np.ndarray:
        inverse_kinematics = self.sim.inverse_kinematics(
            current_joint, 
            target_position, 
            target_orientation)
        return inverse_kinematics
    # 正运动学计算 输入：关节角度，为一个NumPy数组
    def forward_kinematics(self, qpos) -> np.ndarray:
        f_pos = self.sim.forward_kinematics(qpos=qpos)
        return f_pos

# PiH任务设置
class Task(ABC):
    def __init__(self,sim,) -> None:
        self.sim = sim
        self.goal = None  # 初始时将目标设为None

    @abstractmethod
    def reset(self) -> None:
        """Reset the task: sample a new goal.用于重置任务，例如随机选择一个新的目标。"""

    @abstractmethod
    def get_obs(self) -> np.ndarray:
        """Return the observation associated to the task.用于获取与任务相关的观测。"""

    @abstractmethod
    def get_achieved_goal(self) -> np.ndarray:
        """Return the achieved goal.用于获取已实现的目标。"""

    def get_goal(self) -> np.ndarray:
        """Return the current goal.获取当前的目标。"""
        if self.goal is None:
            raise RuntimeError("No goal yet, call reset() first")
        else:
            return self.goal.copy()

    @abstractmethod
    def is_success(
        self, achieved_goal: np.ndarray, desired_goal: np.ndarray, info: Dict[str, Any] = {}
    ) -> Union[np.ndarray, float]:
        """Returns whether the achieved goal match the desired goal.判断已实现的目标是否与期望目标匹配"""

    @abstractmethod
    def compute_reward(
        self, achieved_goal: np.ndarray, desired_goal: np.ndarray, info: Dict[str, Any] = {}
    ) -> Union[np.ndarray, float]:
        """Compute reward associated to the achieved and the desired goal.计算与已实现目标和期望目标相关的奖励值。"""

# 强化学习环境设置 状态空间
class RobotTaskEnv(gym_robotics.GoalEnv):
    # metadata = {"render.modes": ["human", "rgb_array"]}
    # 初始化 机器人和任务模拟环境 设置一些属性，包括目标数量、模拟器、机器人、任务、是否初始化抓取和是否渲染环境
    def __init__(
        self,
        robot: MujocoRobot,
        task: Task,
        # init_grasping: bool,
        render: bool = True,
        num_goal: int = 1,
        real_robot: bool = True,
        ) -> None:
        assert robot.sim == task.sim, "The robot and the task must belong to the same simulation."
        self.num_goal = num_goal
        self._num_goal = 0
        self.sim = robot.sim
        self.robot = robot
        self.task = task
        self.render = render
        # self.init_grasping = init_grasping
        # self._reset_first = False
        self.one_episode = 0
        obs = self._get_obs()  # 初始化所需参数； 随机种子可以稍后更改
        self.i_step = 0  # 0418

        observation_shape = obs.shape
        self.observation_space = spaces.Box(-1, 1, shape=observation_shape, dtype=np.float32)
        # observation_shape = obs['robot_task_obs'].shape
        # self.observation_space = spaces.Dict({'robot_task_obs':
        #                                  spaces.Box(low=-1, high=1, shape=observation_shape, dtype=np.float32)})
        print("obs space:",self.observation_space)
        self.action_space = self.robot.action_space
        self.compute_reward = self.task.compute_reward
        self.initial_distance = None  # 初始距离
        self.real_robot = real_robot
    # 环境更新
    def reset(self, seed: Optional[int] = None, options={}) -> Dict[str, np.ndarray]:
        self.one_episode = 0
        self.robot.reset()
        self.task.reset()
        self.initial_distance = self.distance2goal()
        self.sim.set_forward() # 更新环境状态
        info = dict()
        return self._get_obs(), info
    # 环境状态
    def _get_obs(self) -> Dict[str, np.ndarray]:
        robot_obs = self.robot.get_obs()  # 机器人状态
        task_obs = self.task.get_obs()
        observation = np.concatenate([robot_obs, task_obs]).astype(np.float32)

        achieved_goal = self.task.get_achieved_goal()
        # print("状态：",observation)
        # observation = np.concatenate([robot_obs, task_obs,achieved_goal]).astype(np.float32)
        # observation = {
        #     'ee_pos': np.copy(observation[:3]),
        #     'ee_rot': np.copy(observation[3:6]),
        #     'f_sensor': np.copy(observation[6:9]),
        #     't_sensor': np.copy(observation[9:12]),
        #     'hole_bottom_pos': np.copy(observation[12:15]),
        #     'usb_bottom_pos': np.copy(observation[15:18])
        # }
        # # observation = {key: value.reshape(-1) for key, value in observation.items()}
        # return {"robot_task_obs":observation}
        return observation
    # 步进
    def step(self, action: np.ndarray) -> Tuple[Dict[str, np.ndarray], float, bool, Dict[str, Any]]:
        # # 奖励v3
        # done = False
        # #prev_distance2goal = self.distance2goal()
        # #print("prev_distance2goal:",prev_distance2goal)
        # self.robot.set_action(action) # 更新数据
        # obs = self._get_obs()
        # self.one_episode += 1
        # info = {"is_success": self.task.is_success(self.task.get_achieved_goal(), self.task.get_goal())}
        # # print(info)
        # reward1 = self.reward_design(self.task.get_achieved_goal(), self.task.get_goal(), info)
        # cur_ft = np.copy(obs[6:12])
        # reward_scale = 1.0  # 缩放因子
        # # if not np.all((-self.robot.f_sensor_thresh < cur_ft[:3]) & (cur_ft[:3] < self.robot.f_sensor_thresh)):
        # #     self.penalty_ratio += 0.1
        # #     f_reward = -2.5 * self.penalty_ratio
        # # else:
        # #     f_reward = 0
        # #     self.penalty_ratio =1
        # #     # print("cur_ft:",cur_ft)
        # # if not np.all((-self.robot.t_sensor_thresh < cur_ft[3:]) & (cur_ft[3:] < self.robot.t_sensor_thresh)):
        # #     self.penalty_ratio += 0.1
        # #     t_reward = -2.5 * self.penalty_ratio
        # # else:
        # #     t_reward = 0
        # #     self.penalty_ratio =1
        # # reward2 = t_reward * reward_scale + f_reward * reward_scale
        # # reward = reward1 + reward2
        # reward = reward1
        # if self.task.is_success(self.task.get_achieved_goal(), self.task.get_goal()):
        #     #print('判断结果：', self.task.is_success(self.task.get_achieved_goal(), self.task.get_goal()))
        #     self._num_goal += 1
        #     if self._num_goal >= self.num_goal:
        #         done = True
        #         print("----USB插入成功----")
        #         self._num_goal = 0
        #     else:
        #         done = False
        # else:
        #     self._num_goal = 0
        #     done = False
        # if self.one_episode >= 350:
        #     done = True
        # # if not np.all((-self.robot.f_sensor_thresh < cur_ft[:3]) & (cur_ft[:3] < self.robot.f_sensor_thresh)):
        # #     done = True
        # #     print("done =", done)
        # # if not np.all((-self.robot.t_sensor_thresh < cur_ft[3:]) & (cur_ft[3:] < self.robot.t_sensor_thresh)):
        # #     done = True
        # #     print("done =", done)
        # truncated = False
        # return obs, reward, done, truncated, info

        # 奖励v2
        done = False
        prev_distance2goal = self.distance2goal()
        # print("prev_distance2goal:",prev_distance2goal)
        # print("距离t:", prev_distance2goal*1000)
        self.robot.set_action(action) # 更新数据
        obs = self._get_obs()
        # print("状态：",obs)
        self.one_episode += 1
        info ={'is_success': self.success_check()}  # 键值对
        # 距离奖励
        # d_reward = ((prev_distance2goal-self.distance2goal())/self.initial_distance)
        # print("d_reward:",d_reward)
        # 步数惩罚
        step_penalty = -0.01
        # print("cur_distance2goal:",self.distance2goal(),"initial_distance:",self.initial_distance)
        cur_distance = self.distance2goal()
        # print("cur_distance:",cur_distance)
        # 靠近奖励
        if cur_distance < prev_distance2goal:
            approach_reward = 0
        else:
            approach_reward = -1
        # print("距离t+1:", cur_distance *1000)
        # print("prev_distance2goal:",prev_distance2goal," cur_distance2goal:",self.distance2goal())
        # 成功奖励
        if cur_distance<0.0022:
            s_reward = 10
        else:
            s_reward = 0
        #s_reward = self.success_check()
        # 位置超出范围惩罚
        if cur_distance>0.05:
            thresh_reward = -100
        else:
            thresh_reward = 0
        # 力/力矩超出范围惩罚
        # cur_ft = np.copy(obs[6:12])
        cur_ft = self.robot.get_real_ft_sensor()
        #cur_ft_scaled = np.array(cur_ft[3:])/1000.0
        cur_ft_scaled = np.array(cur_ft[3:])
        cur_ft[3:] = cur_ft_scaled.tolist()
        # ft_data = pd.DataFrame(cur_ft)
        # ft_data.to_excel("/home/wfh/PiH-RL/sonser_data/ft_data.xlsx",index=False)
        # print(cur_ft)
        if not np.all((-self.robot.f_sensor_thresh < cur_ft[:3]) & (cur_ft[:3] < self.robot.f_sensor_thresh)):
            print("当前力超过阈值:",cur_ft)
        if not np.all((-self.robot.t_sensor_thresh < cur_ft[3:]) & (cur_ft[3:] < self.robot.t_sensor_thresh)):
            print("当前力矩超过阈值:",cur_ft)
        #cur_ft = np.copy(obs['robot_task_obs'][6:12])
        #print("cur_ft:",cur_ft)
        f_reward = 0 if np.all((-self.robot.f_sensor_thresh < cur_ft[:3]) & (cur_ft[:3] < self.robot.f_sensor_thresh)) else -10
        t_reward = 0 if np.all((-self.robot.t_sensor_thresh < cur_ft[3:]) & (cur_ft[3:] < self.robot.t_sensor_thresh)) else -10
        #print(f"f_reward = {f_reward:.2f} t_reward = {t_reward:.2f}")
        # 归一化和缩放奖励
        '''
        reward_scale = 0.01  # 缩放因子
        reward_offset = 0.1  # 偏移量

        d_reward = (d_reward - reward_offset) * reward_scale
        step_penalty = step_penalty * reward_scale
        approach_reward = approach_reward * reward_scale
        s_reward = s_reward * reward_scale
        thresh_reward = thresh_reward * reward_scale
        f_reward = f_reward * reward_scale
        t_reward = t_reward * reward_scale
        '''
        # 总奖励
        # reward = d_reward + step_penalty + approach_reward + s_reward + thresh_reward
        #reward = d_reward + step_penalty + approach_reward + s_reward + thresh_reward
        # reward = d_reward + s_reward
        reward = approach_reward + s_reward
        # print("奖励：",reward)
        #print(f"d_reward = {d_reward:.2f} s_reward = {s_reward:.2f} reward = {reward:.2f}")
        if self.success_check():
            self._num_goal += 1
            if self._num_goal >= self.num_goal:
                print("-------USB插入成功--------")
                done =True
            else:
                done = False

        else:
            # done = False if np.all((-self.robot.f_sensor_thresh < cur_ft[:3]) & (cur_ft[:3] < self.robot.f_sensor_thresh)) else True
            # done = False if np.all((-self.robot.t_sensor_thresh < cur_ft[3:]) & (cur_ft[3:] < self.robot.t_sensor_thresh)) else True
            # done = True if (np.any(np.abs(cur_ft[:3]) > self.robot.f_sensor_thresh) or np.any(np.abs(cur_ft[3:]) > self.robot.t_sensor_thresh)) else True
            if not np.all((-self.robot.f_sensor_thresh < cur_ft[:3]) & (cur_ft[:3] < self.robot.f_sensor_thresh)):
                done =True
                print("力超范围了","力：",cur_ft[:3])
            if not np.all((-self.robot.t_sensor_thresh < cur_ft[3:]) & (cur_ft[3:] < self.robot.t_sensor_thresh)):
                done =True
                print("力矩超范围了","力矩：",cur_ft[3:])
            # print("cur_ft:",cur_ft)
            # print("f_sensor_thresh:",self.robot.f_sensor_thresh,"t_sensor_thresh:",self.robot.t_sensor_thresh)
            # 控制步数 350
            if self.one_episode > 250:
                done =True
                print("步数超范围了")

            # 控制范围
            elif self.distance2goal()>0.05:
                done =True
                print("距离超范围了","距离：",self.distance2goal())
        # print("done =", done)
        truncated = False
        return obs, reward, done, truncated, info

        """
        # 奖励v1
        self.robot.set_action(action)
        # self.sim.step()
        obs = self._get_obs()
        done = False
        info = {"is_success": self.task.is_success(self.task.get_achieved_goal(), self.task.get_goal())}
        # TODO:力传感器模式actile sensor
        reward = self.task.compute_reward(self.task.get_achieved_goal(), self.task.get_goal(), info)
        print('奖励计算：', reward)
        assert isinstance(reward, float)  # 检查奖励数值类型
        self.one_episode += 1

        if self.task.is_success(self.task.get_achieved_goal(), self.task.get_goal()):
            print('判断结果：', self.task.is_success(self.task.get_achieved_goal(), self.task.get_goal()))
            self._num_goal += 1
            if self._num_goal >= self.num_goal:
                done = True
                print("插入成功")
                self._num_goal = 0
            else:
                done = False
        else:
            self._num_goal = 0
            done = False
        if self.one_episode >= 350:
            done = True

        truncated = False
        return obs, reward, done, truncated, info
        """
            # 距离计算
    def distance2goal(self):
        if self.real_robot == True:
            obj_bottom,_ = np.copy(self.robot.get_obj_pos())
            # print("usb_pos：",obj_bottom)
            # obj_bottom = np.copy(self.robot.get_obs()[0:3])
            obj_bottom[2] = obj_bottom[2] + 0.0915
            obj_bottom[1] = -obj_bottom[1]
            obj_bottom[2] = -obj_bottom[2]
            # target_bottom = np.copy(self.task.reset())
            # print("当前处的位置：",obj_bottom)
            target_bottom = np.array([0.475-0.0002, -0.050-0.0005, -1.003-0.1035-0.001])
            # print("距离处的位置：",target_bottom)
            # print("usb_bottom:",obj_bottom,"target_bottom:",target_bottom)
            b_distance = distance(obj_bottom, target_bottom)
            # print("距离:",b_distance)
            return b_distance
        else:
            target_bottom = np.copy(self.sim.get_site_position("hole_bottom"))
            obj_bottom = np.copy(self.sim.get_site_position("usb_bottom"))
            b_distance = distance(obj_bottom, target_bottom)
            #print("b_distance:",b_distance)
            return b_distance
    # 判断成功
    def success_check(self):
        #success_reward = 0.
        d_bottom = self.distance2goal()
        # s_check = 1 - np.tanh(100 * d_bottom)
        # s_check = np.array(s_check > 0.875, dtype=np.float64)
        s_check = np.array(d_bottom < 0.0022, dtype=np.float64)
        return s_check

    def reward_design(self, achieved_goal, desired_goal, info: Dict[str, Any]) -> Union[np.ndarray, float]:
        # 目标位置和当前状态的信息
        target_bottom = np.copy(self.sim.get_site_position("hole_bottom"))
        target_top = np.copy(self.sim.get_site_position("hole_top"))
        usb_top = np.copy(self.sim.get_site_position("ubs_top"))
        usb_bottom = np.copy(self.sim.get_site_position("ubs_bottom"))
        # ------------------------
        # 对步数比例值进行tanh计算，将其限制在0.3到1之间，并根据 step_gain 进行调整

        # 步数惩罚---走的步数越多 惩罚越大
        # self.i_step -= 0.0005
        self.i_step += 1
        # print(self.i_step)
        total_steps = 350
        step_ratio = math.tanh((self.i_step / total_steps) * 10) + 0.3
        _step_val = - self.i_step / total_steps  # 一个回合350步
        # step_ratio = math.tanh(self.i_step / 100)
        step_val = _step_val * step_ratio
        # print(step_val)

        # 步数惩罚--tanh归一化 走的步数越多 惩罚越大
        # if self.i_step >= total_steps-20:
        if self.i_step >= total_steps-150:
            step_gain = (math.tanh((total_steps - self.i_step) / 10) - 1) / 20
        else:
            step_gain = 0

        # 距离奖励---三类不同权重的奖励值
        # 定义权重因子，进行奖励缩放
        scaled_ratio_top = np.array([2, 2, 1]) #np.array([1.25, 1.25, 1])
        scaled_ratio_mid = np.array([1, 1, 1])
        scaled_ratio_bot = np.array([1, 1, 1.25]) #np.array([1, 1, 1.25])

        d_top = distance(achieved_goal * scaled_ratio_top, target_top * scaled_ratio_top)
        d_center = distance(achieved_goal * scaled_ratio_mid, desired_goal * scaled_ratio_mid)
        d_bottom = distance(achieved_goal * scaled_ratio_bot, target_bottom * scaled_ratio_bot)
        r_top = (1 - math.tanh(50 * d_top)) * 0.2
        r_center = (1 - math.tanh(30 * d_center)) * 0.3
        r_bot = (1 - math.tanh(100 * d_bottom))

        _r_top = -d_top * 0.1
        _r_center = -d_center * 0.15
        _r_bot = -d_bottom * 1.5
        r_d = (_r_top + _r_center + _r_bot)

        # 成功奖励/失败惩罚---# 判断是否成功插入目标孔
        if r_bot > 0.865:
            # 增加一个成功率的系数
            self.suc_ratio += 0.5
            get_suc = 5 * self.suc_ratio
        else:
            self.suc_ratio = 1
            get_suc = 0

        # 距离奖励---轴孔距离奖励 距离越小奖励越大 （0-0.1）
        scaled_ratio_toptop = np.array([8, 8, 0.1])  # xyz权重 xy平面距离影响大
        d_objtop_holetop = distance(usb_top * scaled_ratio_toptop, target_top * scaled_ratio_toptop)
        r_objtop_holetop = (1 - math.tanh(20 * d_objtop_holetop)) * 0.1

        # 步数惩罚---根据剩余步数、阈值步数来计算惩罚值 剩余步数越低于阈值，惩罚越高 剩余步数大于阈值无惩罚（-0.995~0）
        left_step = total_steps - self.i_step  #总步数350
        threashold_step = 200
        if left_step < threashold_step:
            step_v = (left_step-threashold_step) / (threashold_step * 1)
        else:
            step_v = 0

        return r_d + get_suc + step_gain