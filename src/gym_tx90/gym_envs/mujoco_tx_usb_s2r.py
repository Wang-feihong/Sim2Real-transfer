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
            domain_random: bool = False
    ) -> None:
        self.domain_random = domain_random
        file_root = "/home/wfh1/桌面/TX_PiH_DQN_Real/gym_envs/models/"
        # txDH_file = "/home/wfh/桌面/TX_PiH_DQN_Real/gym_envs/models/tx90v1.urdf"
        txDH_file = "/home/wfh1/桌面/TX_PiH_DQN_Real/gym_envs/models/mujoco_tx90.urdf"
        self.tx90kdl = txkdl.TX_kdl(txDH_file)
        print("--------完成加载URDF文件--------")
        if self.domain_random == True:
            xml_file = 'tx_pih0619.xml'
        else:
            xml_file = 'tx_pih0511.xml'
        self.xml_file = file_root + xml_file
        print("xml_file:", self.xml_file)
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
        # self.tool_stiffness_range = np.array([0.01, 0.1])
        # self.tool_damper_range = np.array([0.003, 0.05])
        self.n_substeps = 10
        self.timestep = 0.001
        self.stiffness_range = np.array([5, 70]) #np.array([0.00001, 0.0007]) 
        self.damper_range = np.array([0.15, 5])

    def randomization_xml(self):
        try:
            import xml.etree.cElementTree as ET
        except ImportError:
            import xml.etree.ElementTree as ET
        tool_xml = self.file_root + 'robot0619.xml'
        tree = ET.parse(tool_xml)
        root = tree.getroot()
        # 工具刚度和阻尼的域随机化
        tool_stiffness = str(np.random.uniform(low=self.stiffness_range[0], high=self.stiffness_range[1]))
        tool_damper = str(np.random.uniform(low=self.damper_range[0], high=self.damper_range[1]))
        print("stiffness:",tool_stiffness, " damper:", tool_damper)
        # 文件写入
        i = 0
        for student in root.iter('joint'):
            i += 1
            if i == 2:
                student.set("stiffness", tool_stiffness)
                student.set("damping", tool_damper)
        tree.write(tool_xml)
        time.sleep(0.01)  # 等待文件写入
        tool_xml = self.file_root + 'hole0511.xml'
        tree = ET.parse(tool_xml)
        root = tree.getroot()  # 获取根节点
        rgba = " ".join(str(x) for x in np.random.uniform(low=np.array([0.1, 0.1, 0.1, 0.8]), high=np.array([0.9, 0.9, 0.9, 1])))
        print("rgba:", rgba)
        for geom in root.iter('geom'):
                geom.set("rgba", rgba)
        tree.write(tool_xml)
        time.sleep(0.01)  # 等待文件写入
        self.model = mujoco.MjModel.from_xml_path(self.xml_file)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        if self.render:
            if hasattr(self, 'viewer'):  # 检查对象 self 是否具有名为 'viewer' 的属性
               self.viewer.close()  # 关闭之前的窗口
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)

    @property
    def dt(self):
        return self.timestep * self.n_substeps

    def reset(self) -> None:
        if self.domain_random == True:
            self.randomization_xml()
        print("is_域随机化：", self.domain_random)
        mujoco.mj_resetData(self.model, self.data)
        if self.domain_random == True:
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
        # mujoco.mj_resetDataKeyframe(self.model, self.data, key=0)
        # time.sleep(1)
        # mujoco.mj_resetDataKeyframe(self.model, self.data, key=1)
        # time.sleep(2)

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

test1 = False
print("func_test1中...")
if test1 is True:
    test_env = Mujoco_Func()
    fw_qpos = np.array([6.31184355e-05,  1.57074928e+00, -1.57037824e+00,  3.43973606e-04,-2.11636511e-03 , 1.57038923e+00])
    # fw_qpos = np.array([0, 90*np.pi/180, -90*np.pi/180, 0, 0, 0]) # -0.25973099  0.17732469 -0.19064892  2.12031658 -0.01493478 -4.71238898
    #fw_qpos = np.array([-0.25973099,  0.17732469, -0.19064892,  2.12031658, -0.01493478, -4.71238898])
    #fw_qpos = np.array([3.73766532e-03,  1.59581573e-01, -1.74521914e-01, -4.71238898e+00,-1.49400789e-02, 3.13885709e+00])
    # fw_qpos = np.array([0, 0, 0, 0, 0, 0])
    # fw_qpos = [-3.31677747e-06,  1.17649865e-02, -1.18319315e-02, -2.16010142e-01,-6.11919876e-05,  2.16013459e-01]
    # fw_qpos = [-8.74968556e-06, -3.52959862e-03,  3.52799774e-03,  6.25816449e-03,5.85900520e-06, -6.24941482e-03] # z向上移动1mm
    # fw_qpos = [-8.69181586e-06, -5.88256809e-03,  5.86935688e-03,  4.34838670e-06,-5.86479101e-06,  4.34333211e-06]  # z向上移动2mm
    # fw_qpos = [1.30569340e+00, -2.37184453e-01,  1.78233439e+00,  7.21431081e-09,-1.54514981e+00, -1.30569396e+00]
    # fw_qpos = [-1.57079633, -0.01002289, -1.41907302 , 3.14159265, -1.4290959,   4.71238898]
    # fw_qpos = [1.29771202, -0.02251418,  0.9268447,  -3.10455742, -2.00712864, -4.71238898]
    joint_name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    current_arm_joint = np.zeros(6)
    # current_arm_joint = np.array([0, -1.57, 1.57, 0, 0, 0])
    test_env.reset()
    i=0
    while i < 2000:
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
        # print(fw_qpos)
        # fo1=test_env.forward_kinematics(fw_qpos)
        # current_joint = np.copy([test_env.get_joint_angle(joint_name[i]) for i in range(6)])
        # ik=test_env.inverse_kinematics(current_joint,fo1[0],fo1[1])
        # fo2=test_env.forward_kinematics(ik)
        # print("正解：",fo1)
        print("当前关节角：",current_arm_joint)
        # print("逆解：",ik)
        # ik = test_env.forward_kinematics(current_arm_joint)
        print('wrist_3_link:',test_env.get_body_position("ee_link"))
        print('六轴正解末端位置:',test_env.forward_kinematics(current_arm_joint))  # x同 y反 z反且和为2.08
        print("正解姿态：",R.from_quat(test_env.forward_kinematics(current_arm_joint)[1]).as_euler("xyz",degrees=True))
        print('usb_bottom:',test_env.get_site_position("usb_bottom"))
        print("hole_object:",test_env.get_body_position("hole_object"))
        print('object2:',test_env.get_body_position("object2"))
        print('hole_top:',test_env.get_site_position("hole_top"))
        print('hole_center_top:',test_env.get_site_position("hole_center_top"))
        print('hole_center_bottomm:',test_env.get_site_position("hole_center_bottom"))
        print('hole_bottom:',test_env.get_site_position("hole_bottom"))
        print("逆解：", test_env.inverse_kinematics(current_arm_joint,np.array([0.475, 0.05, 1.003]),np.array([0, 0, 0, 1])))
        fw_qpos_c= np.array([0, 0, 0, 0, 0, 0])
        target_pos= [-0.375,  0.04999,  1.0025]
        target_rot= [0,0,1,0]
        # print("误差：",test_env.calculate_error(fw_qpos_c,target_pos,target_rot))

# simulate and render
test2 = False
if test2 is True:
    i=0
    testkey = Mujoco_Func()
    testkey.reset()
    while i < 500:
        i += 1
        testkey.step()
        # mujoco.mj_step(testkey.model, testkey.data)
        testkey.viewer.render()
        # testkey.reset()
    # close
    testkey.viewer.close()
