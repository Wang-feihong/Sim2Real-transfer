'''
Name:
Date: 2024-04-18 14:53:33
Creator: 王飞鸿
Description:
'''
import time
from contextlib import contextmanager
from typing import Any, Dict, Iterator, Optional
import numpy as np
import gym_envs.envs.tx90_kdl as txkdl
import mujoco
import mujoco_viewer
from scipy.spatial.transform import Rotation as R

class Mujoco_Func:
    # mujoco基础功能构建
    def __init__(
            self,
            render: bool = True,
            real_robot: bool = False,
            file_root="/home/wfh/catkin_ws/src/gym_tx90/gym_envs/models/",
    ) -> None:
        if real_robot == True:
            txDH_file = "/home/wfh/catkin_ws/src/gym_tx90/gym_envs/models/mujoco_tx90.urdf"
        # else:
            # txDH_file = "/home/wfh/catkin_ws/src/gym_tx90/gym_envs/models/tx90v1.urdf"
        self.tx90kdl = txkdl.TX_kdl(txDH_file)
        print("--------完成加载URDF文件--------")
        xml_file = 'tx_pih0418.xml'  # USB
        # xml_file = 'tx_pih0809_rect.xml'
        # xml_file = 'tx_pih0811_squa.xml'
        # xml_file = 'tx_pih0811_doub.xml'
        # xml_file = 'tx_pih0828_six.xml'
        self.xml_file = file_root + xml_file
        self.model = mujoco.MjModel.from_xml_path(self.xml_file)  # xml路径
        self.data = mujoco.MjData(self.model)  # 数据转化
        mujoco.mj_forward(self.model, self.data)
        if render:
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
            self.viewer.add_line_to_fig(line_name="force-x", fig_idx=1)
            self.viewer.add_line_to_fig(line_name="force-y", fig_idx=1)
            self.viewer.add_line_to_fig(line_name="force-z", fig_idx=1)
            self.viewer.add_line_to_fig(line_name="torque-x", fig_idx=0)
            self.viewer.add_line_to_fig(line_name="torque-y", fig_idx=0)
            self.viewer.add_line_to_fig(line_name="torque-z", fig_idx=0)
            # print("sensor-xyz:",self.data.sensor("ee_force_sensor"))
            # print("sensor-xyz:",self.data.sensor("ee_torque_sensor"))
            fig = self.viewer.figs[1]
            fig.title = "Force-xyz"
            fig.flg_legend = True
            fig.xlabel = "Timesteps"
            fig.figurergba[0] = 0.2
            fig.figurergba[1] = 0.5
            fig.figurergba[3] = 0.0
            fig.gridsize[0] = 8
            fig.gridsize[1] = 5
            fig = self.viewer.figs[0]
            fig.title = "Torque-xyz"
            fig.flg_legend = True
            fig.xlabel = "Timesteps"
            fig.figurergba[0] = 0.2
            fig.figurergba[1] = 0.5
            fig.figurergba[3] = 0.0
            fig.gridsize[0] = 8
            fig.gridsize[1] = 5
            # self.viewer.add_data_to_line(line_name="ee_force_sensor", line_data=self.data.sensor("ee_force_sensor").data,fig_idx=1)
        self.render = render
        self.file_root = file_root
        # self.stiffness_range = np.array([0.01, 0.1])
        # self.damper_range = np.array([0.003, 0.05])
        self.n_substeps = 10
        self.timestep = 0.001

    @property
    def dt(self):
        return self.timestep * self.n_substeps

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
        if self.render is True:
            self.viewer.render()
            self.viewer.add_data_to_line(line_name="force-x", line_data=self.data.sensor("ee_force_sensor").data[0],fig_idx=1)
            self.viewer.add_data_to_line(line_name="force-y", line_data=self.data.sensor("ee_force_sensor").data[1],fig_idx=1)
            self.viewer.add_data_to_line(line_name="force-z", line_data=self.data.sensor("ee_force_sensor").data[2],fig_idx=1)
            self.viewer.add_data_to_line(line_name="torque-x", line_data=self.data.sensor("ee_torque_sensor").data[0],fig_idx=0)
            self.viewer.add_data_to_line(line_name="torque-y", line_data=self.data.sensor("ee_torque_sensor").data[1],fig_idx=0)
            self.viewer.add_data_to_line(line_name="torque-z", line_data=self.data.sensor("ee_torque_sensor").data[2],fig_idx=0)

    def get_body_position(self, body: str) -> np.ndarray:
        position = self.data.xpos[mujoco.mj_name2id(self.model, type=1, name=body)]
        return np.array(position)

    def get_body_quaternion(self, body: str) -> np.ndarray:
        quat = self.data.xquat[mujoco.mj_name2id(self.model, type=1, name=body)]
        return np.array(quat)

    def get_body_velocity(self, body: str) -> np.ndarray:
        vel = self.data.cvel[mujoco.mj_name2id(self.model, type=1, name=body)]
        return np.array(vel)

    def get_joint_angle(self, joint: str) -> float:
        return self.data.qpos[mujoco.mj_name2id(self.model, type=3, name=joint)]

    def get_joint_velocity(self, joint: str) -> float:
        return self.data.qvel[mujoco.mj_name2id(self.model, type=3, name=joint)]

    def get_site_position(self, site: str) -> np.ndarray:
        return self.data.site_xpos[mujoco.mj_name2id(self.model, type=6, name=site)]

    def get_site_mat(self, site: str) -> np.ndarray:
        return self.data.site_xmat[mujoco.mj_name2id(self.model, type=6, name=site)]

    def set_joint_angles(self, angles: np.ndarray) -> None:
        for i in range(len(angles)):
            self.data.qpos[i] = angles[i]
        mujoco.mj_forward(self.model, self.data)

    def set_mocap_pos(self, mocap: str, pos: np.ndarray) -> None:
        self.data.mocap_pos[0] = pos  # TODO:mujoco中没有定义id，需要设计一个搜索方法
        # self.data.mocap_pos[mujoco.mj_name2id()]

    def set_mocap_quat(self, mocap: str, quat: np.ndarray) -> None:
        self.data.mocap_quat[0] = quat  # TODO: the same problem like set_mocap_pos func

    # 关节控制
    def control_joints(self, target_angles: np.ndarray) -> None:
        for i in range(len(target_angles)):
            self.data.ctrl[i] = target_angles[i]

    def set_forward(self) -> None:
        mujoco.mj_forward(self.model, self.data)

    # def get_touch_sensor(self, sensor: str) -> float:
    # return self.data.sensor(sensor)

    def get_ft_sensor(self, force_site: str, torque_site: str) -> np.ndarray:
        force = self.data.sensor(force_site).data
        torque = self.data.sensor(torque_site).data
        return np.hstack((force, torque))

    def inverse_kinematics(self, current_joint: np.ndarray, target_position: np.ndarray,
                           target_orientation: np.ndarray) -> np.ndarray:
        qpos = self.tx90kdl.inverse(current_joint, target_position, target_orientation)
        return qpos

    def forward_kinematics(self, qpos) -> np.ndarray:
        ee_pos = self.tx90kdl.forward(qpos=qpos)
        return ee_pos

    def calculate_error(self, current_joint, target_position, target_orientation):
        # 逆向运动学求解新的关节角度
        new_joint = self.inverse_kinematics(current_joint, target_position, target_orientation)
        # 正向运动学计算新的末端执行器位置
        new_ee_pos = self.forward_kinematics(new_joint)[0]
        # 计算求解误差
        print("new_joint:",new_joint)
        print('new_ee_pos:', new_ee_pos)
        print('target_position:', target_position)
        error = np.linalg.norm(new_ee_pos - target_position)
        return error

    @contextmanager
    def no_rendering(self) -> Iterator[None]:
        pass

test = False
print("func_test中...")
if test is True:
    test_env = Mujoco_Func()
    #fw_qpos = np.array([0.723, 0.314, -1.01, 0, -0.723, 0])
    # fw_qpos = np.array([0, 0, 0, 0, 0, 0]) # -0.25973099  0.17732469 -0.19064892  2.12031658 -0.01493478 -4.71238898
    #fw_qpos = np.array([-0.25973099,  0.17732469, -0.19064892,  2.12031658, -0.01493478, -4.71238898])
    #fw_qpos = np.array([3.73766532e-03,  1.59581573e-01, -1.74521914e-01, -4.71238898e+00,-1.49400789e-02, 3.13885709e+00])
    # fw_qpos = np.array([1.57, 0, 0, 0, 0, 0])
    # fw_qpos = [-3.31677747e-06,  1.17649865e-02, -1.18319315e-02, -2.16010142e-01,-6.11919876e-05,  2.16013459e-01]
    # fw_qpos = [-8.74968556e-06, -3.52959862e-03,  3.52799774e-03,  6.25816449e-03,5.85900520e-06, -6.24941482e-03] # z向上移动1mm
    # fw_qpos = [-8.69181586e-06, -5.88256809e-03,  5.86935688e-03,  4.34838670e-06,-5.86479101e-06,  4.34333211e-06]  # z向上移动2mm
    # fw_qpos = [-1.40565027e-05, -1.23816434e-01 , 1.16165619e-01,-1.91944221e-03,-7.64346931e-03,  1.93344248e-03] # z向上移动7mm
    fw_qpos = [-1.55285900e-05, -2.43132460e-01,  2.13721836e-01, -4.97921832e-04,-2.94032775e-02,  5.13233595e-04]  # z向上移动12mm
    joint_name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    current_arm_joint = np.zeros(6)
    test_env.reset()
    i=0
    while i < 10:
        i += 1
        test_env.step()
        test_env.control_joints(fw_qpos)
        for j in range(6):
            current_arm_joint[j] = np.copy(test_env.get_joint_angle(joint_name[j]))
            # print('当前关节角{}：{}'.format(j + 1, current_arm_joint[j]))
            # print('当前关节角%d：%s' % (j + 1, current_arm_joint[j]))
        # r = R.from_matrix(test_env.get_site_mat('attachment_site').reshape(3, 3))  # 将位姿矩阵转换为旋转矩阵
        # r = R.from_matrix(test_env.get_site_mat('ee_site').reshape(3, 3))  # 将位姿矩阵转换为旋转矩阵
        # r = test_env.get_body_position("wrist_3_link")
        # site_pos = test_env.get_site_position("attachment_site")  # 获取目标位置
        #time.sleep(1)
        print('wrist_3_link:',test_env.get_body_position("wrist_3_link"))
        print('六轴正解末端位置:',test_env.forward_kinematics(fw_qpos))  # x同 y反 z反且和为2.08
        print('usb_bottom:',test_env.get_site_position("usb_bottom"))
        print("hole_object:",test_env.get_body_position("hole_object"))
        print('object2:',test_env.get_body_position("object2"))
        print('hole_top:',test_env.get_site_position("hole_top"))
        print('hole_center_top:',test_env.get_site_position("hole_center_top"))
        print('hole_center_bottomm:',test_env.get_site_position("hole_center_bottom"))
        print('hole_bottom:',test_env.get_site_position("hole_bottom"))
        fw_qpos_c= np.array([0, 0, 0, 0, 0, 0])
        target_pos= [-0.375,  0.04999,  1.0025]
        target_rot= [0,0,1,0]
        print("误差：",test_env.calculate_error(fw_qpos_c,target_pos,target_rot))