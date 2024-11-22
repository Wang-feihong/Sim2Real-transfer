from array import array
from email.mime import base
from typing import Dict, Optional
import numpy as np
import math
from gym import spaces
from gym_envs.envs.real_core2usb import MujocoRobot
from datetime import datetime
from ctypes import util
import time
import numpy as np
from gym_envs.utils import distance, normalizeVector, euler_to_quaternion
from scipy.spatial.transform import Rotation as R
import math

import rospy, sys
import moveit_commander
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from staubli_tx90_planning.msg import Coordinate_force
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy
import random
from staubli_tx90_planning.msg import Coordinate_force
import pandas as pd
import os

class GetCoordinateForce():
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('Get_Coordinate', anonymous=True)

        self.pub = rospy.Publisher('Coordinate_force', Coordinate_force)

        self.force = [0, 0, 0]

        self.torque = [0, 0, 0]

        rospy.Subscriber("optoforce_node/wrench_HEXEA239", WrenchStamped, self.callback)

        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化需要使用move_group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('tx90_arm')

        self.pi = 3.1415926535

    # 计算机器人当前位置（位置+姿态）
    def Compute(self):
        pose = self.arm.get_current_pose()
        (roll, pitch, yaw) = euler_from_quaternion([pose.pose.orientation.x,
                                                    pose.pose.orientation.y,
                                                    pose.pose.orientation.z,
                                                    pose.pose.orientation.w],
                                                   axes='szyx')  # Radians
        (Rz, Ry, Rx) = (roll * 180 / self.pi,
                        pitch * 180 / self.pi,
                        yaw * 180 / self.pi)  # Degrees
        pose = [round(pose.pose.position.x , 5),
            round(pose.pose.position.y , 5),
            round(pose.pose.position.z - 0.00, 5),
            round(Rx, 2),
            round(Ry, 2),
            round(Rz, 2)]
        # pose = [round(pose.pose.position.x , 5),
        #         round(pose.pose.position.y , 5),
        #         round(pose.pose.position.z - 0.478, 5),
        #         round(Rx, 2),
        #         round(Ry, 2),
        #         round(Rz, 2)]
        # pose = [round(pose.pose.position.x , 5),
        #         round(pose.pose.position.y , 5),
        #         round(pose.pose.position.z - 0.478, 5),
        #         round(yaw, 2),
        #         round(pitch, 2),
        #         round(roll, 2)]
        return pose

    # 获取力/力矩
    def callback(self, data):
        # 获取力/力矩数据 数据保留小数点后两位
        self.force = [round(data.wrench.force.x, 2),
                      round(data.wrench.force.y, 2),
                      round(data.wrench.force.z, 2)]
        self.torque = [round(data.wrench.torque.x, 2),
                       round(data.wrench.torque.y, 2),
                       round(data.wrench.torque.z, 2)]

class TX_90(MujocoRobot):
    def __init__(self, sim,_normalize,real_robot: bool = False,) -> None:
        # n_action = 6
        n_action = 12
        pxzy = 0.0002
        rxzy = 0.1 #0.05
        # action_space = spaces.Box(-1, 1, shape=(n_action,), dtype=np.float32)
        action_space = spaces.Discrete(n_action)
        self._action_to_direction = {
            0: np.array([pxzy,0,0]),
            1: np.array([-pxzy,0,0]),
            2: np.array([0,pxzy,0]),
            3: np.array([0,-pxzy,0]),
            4: np.array([0,0,pxzy]),
            5: np.array([0,0,-pxzy-0.0002]), #xyz位移
            6: np.array([rxzy,0,0]),
            7: np.array([-rxzy,0,0]),
            8: np.array([0,rxzy,0]),
            9: np.array([0,-rxzy,0]),
            10: np.array([0,0,rxzy]),
            11: np.array([0,0,-rxzy]), #xyz旋转
        }
        print("action_space",action_space)
        # self.force_threshold = True
        # self.ee_pos_low = np.array([-0.39, 0.04, 0.95]) #np.array([-0.4, 0.2, 0.5])  # 旋转90度 6轴末端 [ 0.04969808 -0.37503659  1.07749758]for real 0604关
        # self.ee_pos_high = np.array([-0.36,  0.06, 1.5]) #np.array([0,  0.45, 1.2])  #0.36-0.39 0.04-0.06
        # self.ee_pos_low = np.array([-1, -1, 0.95]) #np.array([-0.4, 0.2, 0.5])  # 旋转90度 6轴末端 [ 0.04969808 -0.37503659  1.07749758]
        # self.ee_pos_high = np.array([1,  1, 1.5]) #np.array([0,  0.45, 1.2]) #0429关
        # self.ee_pos_low = np.array([0.470, 0.04, 1.000]) #np.array([-0.4, 0.2, 0.5])  # for real
        # self.ee_pos_high = np.array([0.480,  0.06, 1.015]) #np.array([0,  0.45, 1.2])
        self.ee_pos_low = np.array([0.470, 0.04, 1.000]) #np.array([-0.4, 0.2, 0.5])  # for real
        self.ee_pos_high = np.array([0.480,  0.06, 1.015]) #np.array([0,  0.45, 1.2])
        self.ee_rot_low = np.array([-10, -10, -180])
        self.ee_rot_high = np.array([10, 10, 180])
        ft_xy = 20
        ft_xyz = 20
        ft_rxyz = 2
        self.ft_sensor_low = np.array([-ft_xy, -ft_xy, -ft_xyz, -ft_rxyz, -ft_rxyz, -ft_rxyz])
        self.ft_sensor_high = np.array([ft_xy, ft_xy, ft_xyz, ft_rxyz, ft_rxyz, ft_rxyz])
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
        self.low_filter_gain = 0.1
        self._normalize_obs =_normalize

        self.real_robot = real_robot
        self.gcf = GetCoordinateForce()
        # 发布力传感器节点话题
        self.pub = rospy.Publisher('/optoforce_node/reset', Empty)
        # self.pub = rospy.Subscriber('/joint_states', self.callback())
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('tx90_arm')
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.00003)
        self.arm.set_goal_orientation_tolerance(0.003)
        self.pi = 3.1415926535
        self.d2r = self.pi/180
        self.file_counter = 0
        self.pih_eval = []
        # self.action_eval = []
        self.action_ = None
        super().__init__(
            sim,
            action_space=action_space,
            joint_index=np.array([0, 1, 2, 3, 4, 5, 6]),
            #joint_forces=np.array([87.0, 87.0, 87.0, 87.0, 12.0, 120.0, 170.0]),
            joint_list=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
            sensor_list=["touchsensor_r1", "touchsensor_r2", "touchsensor_r3", "touchsensor_r4", "touchsensor_r5", "touchsensor_r6",
                "touchsensor_l1", "touchsensor_l2", "touchsensor_l3", "touchsensor_l4", "touchsensor_l5", "touchsensor_l6"]
            )
        if self.real_robot == True:
            # 设置机械臂的初始位置，使用六轴的位置数据进行描述（单位：弧度）
            self.init_joint = np.array([0.*self.d2r, 90.*self.d2r, -90.*self.d2r, 0.*self.d2r, 0.*self.d2r, 90.0*self.d2r])
            # self.init_joint = np.array([0, 1.57, -1.57, 0., 0., 0.])
            self.arm.set_joint_value_target(self.init_joint)
            # 控制机械臂完成运动
            self.arm.go()
            rospy.sleep(5)
            # 初始位置设置方式2
            # self.arm.set_named_target('home_1')
            # self.arm.go()
            # rospy.sleep(2)
        else:
            #self.init_joint = np.array([0.754, 0.66, -1.54, 0, -0.88, 0]) #面上
            #self.init_joint = np.array([0.723, 0.314, -1.01, 0, -0.723, 0]) #上方
            #self.init_joint = np.array([1.57, 0., 0., 0., 0., 0]) # 正面 旋转90
            self.init_joint = np.array([0., 0., 0., 0., 0., 90.])

    # def callback(self):
    #     msg = Coordinate_force()
    #     joint_states = msg.position

    # 运动规划 execute运动
    def cartesian_move(self, waypoints, wait_time):
        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints,  # waypoint poses，路点列表
                0.0001,  # eef_step，终端步进值
                0.0,  # jump_threshold，跳跃阈值
                True)  # avoid_collisions，避障规划
            # 尝试次数累加
            attempts += 1

            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")

            self.arm.execute(plan)

            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) +
                          " success after " + str(maxtries) + " attempts.")

        rospy.sleep(wait_time)

    def move_pose(self, x, y, z, Rx, Ry, Rz):
        # 获取当前位姿数据作为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        # 初始化路点列表
        waypoints = []
        # 设置第二个路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        # target_ee_rot = [Rx, Ry, Rz]
        pi = 3.1415926535
        (roll, pitch, yaw) = (Rz * pi / 180,
                            Ry * pi / 180,
                            Rx * pi / 180)
        (wpose.orientation.x, wpose.orientation.y, wpose.orientation.z,wpose.orientation.w) = quaternion_from_euler(roll, pitch, yaw, axes='szyx')
        # (wpose.orientation.x, wpose.orientation.y, wpose.orientation.z,wpose.orientation.w) = R.from_euler('xyz', target_ee_rot, degrees=True).as_quat()

        waypoints.append(deepcopy(wpose))
        self.cartesian_move(waypoints, 5)

    def target_move(self,target_ee_pos, target_ee_eul):
        print ("[INFO] execute Target Move")
        self.move_pose(target_ee_pos[0], target_ee_pos[1],
                            target_ee_pos[2] + 0.00, target_ee_eul[0],
                            target_ee_eul[1], target_ee_eul[2])
        # self.move_pose(target_ee_pos[0], target_ee_pos[1],
        #             target_ee_pos[2] + 0.478, target_ee_eul[0],
        #             target_ee_eul[1], target_ee_eul[2])
        self.pub.publish()

    def get_obj_pos(self):
        obj_pos = self.gcf.Compute()[:3]
        obj_eul = self.gcf.Compute()[3:]
        return obj_pos,obj_eul

    def get_real_ft_sensor(self):
        # 力/力矩
        msg = Coordinate_force()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "Coordinate_force"
        msg.wrench.force.x = self.gcf.force[0]
        msg.wrench.force.y = self.gcf.force[1]
        msg.wrench.force.z = self.gcf.force[2]
        msg.wrench.torque.x = self.gcf.torque[0]/1000
        msg.wrench.torque.y = self.gcf.torque[1]/1000
        msg.wrench.torque.z = self.gcf.torque[2]/1000
        self.gcf.pub.publish(msg)
        ft_sensor = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        return ft_sensor

    # 动作设计
    def set_action(self, action:np.ndarray) -> None:
        self.action_ = action
        _action = self._action_to_direction[action]
        # self.save_data_action(action)
        # action = np.clip(action, self.action_space.low, self.action_space.high)
        displacement = np.copy(_action)
        # print("displacement1:",displacement)
        if 0 <= action <= 5:
            ee_displacement = np.concatenate((displacement, np.zeros(3)))
        else:
            ee_displacement = np.concatenate((np.zeros(3), displacement))
        # print("动作：",ee_displacement)
        if self.real_robot ==True:
            # target_ee_pos, target_ee_eul = self.ee_displacement_to_target_arm_angles(ee_displacement)
            # print("Next位置：",target_ee_pos,"Next姿态：",target_ee_eul )
            # self.target_move(target_ee_pos, target_ee_eul)
            # print("运动后位置：",self.gcf.Compute()[:3])
            # print("运动后姿态：",self.gcf.Compute()[3:])
            # current_joint_arm = self.arm.get_current_joint_values()
            # print("Next_joint:",current_joint_arm)
            start_pos_rot = self.gcf.Compute()
            target_arm_angles = self.ee_displacement_to_target_arm_angles(ee_displacement)
            # print("目标关节角：",target_arm_angles)
            self.arm.set_joint_value_target(target_arm_angles)
            self.arm.go()
            rospy.sleep(2)
            # print("运动后位置_直接获取：",self.gcf.Compute()[:3],"运动后姿态_直接获取：",self.gcf.Compute()[3:])
            # print("运动后位置_关节角计算：",self.forward_kinematics(self.arm.get_current_joint_values())[0],"运动后姿态_关节角计算：",R.from_quat(self.forward_kinematics(self.arm.get_current_joint_values())[1]).as_euler("xyz",degrees=True))
            time.sleep(1)
            # print("pos:",(start_pos_rot+ee_displacement-self.gcf.Compute())[:3])
            # print("运动误差:",[x*1000 for x in (start_pos_rot+ee_displacement-self.gcf.Compute())[:3]])
        else:
            target_arm_angles = self.ee_displacement_to_target_arm_angles(ee_displacement)
            current_joint_arm = np.array([self.get_joint_angle(joint=self.joint_list[i]) for i in range(6)])
            if np.isnan(target_arm_angles).any() is True or np.isfinite(target_arm_angles).all() is False:
                target_arm_angles = np.copy(current_joint_arm)
            # print("test_pos",test_pos_rot[0],"test_rot:",R.from_quat(test_pos_rot[1]).as_euler("xyz",degrees=True))
            self.joint_planner(_time=self.planner_time/50,
                        current_joint=current_joint_arm,
                        target_joint=target_arm_angles)

    def save_data_action(self, _action):

        # 创建DataFrame
        self.action_eval.append({'action': _action})
        df = pd.DataFrame(self.action_eval)
        # file_name = f"{self.file_counter}.xlsx"
        # file_name = f"real_evalate_0817{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
        file_name = "action_evalate_0818_10times01.xlsx"
        file_path = os.path.join("/home/wfh/PiH-RL/randomhole/real_pih", file_name)
        df.to_excel(file_path, index=False)
    # 获取观测值
    def get_obs(self) -> float:
        if self.real_robot ==True:
            real_current_joint = self.arm.get_current_joint_values()
            # print("当前关节角:",real_current_joint)
            ee_pos = self.forward_kinematics(real_current_joint)[0]
            # ee_pos[1] = -ee_pos[1]
            # ee_pos[2] = -ee_pos[2]
            ee_eul = R.from_quat(self.forward_kinematics(real_current_joint)[1]).as_euler("xyz",degrees=True)
            # print("当前位姿：",ee_pos,ee_eul)
            # ee_eul[0]=(ee_eul[0]+180)%360
            # print("位置 姿态：",ee_pos,ee_eul)
            # # 位置
            # ee_pos = self.gcf.Compute()[:3]
            # # 姿态
            # ee_eul = self.gcf.Compute()[3:]

            # 力/力矩
            msg = Coordinate_force()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "Coordinate_force"
            msg.wrench.force.x = self.gcf.force[0]
            msg.wrench.force.y = self.gcf.force[1]
            msg.wrench.force.z = self.gcf.force[2]
            msg.wrench.torque.x = self.gcf.torque[0]/1000
            msg.wrench.torque.y = self.gcf.torque[1]/1000
            msg.wrench.torque.z = self.gcf.torque[2]/1000
            self.gcf.pub.publish(msg)
            # time.sleep(0.1)
            ft_sensor = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
            # ft_sensor = self.ft_last_obs * self.low_filter_gain + ft_sensor * (1 - self.low_filter_gain)
            # ft_sensor = np.array(self.ft_last_obs) * self.low_filter_gain + np.array(ft_sensor) * (1 - self.low_filter_gain)
            # print('obs_pos:',ee_pos)
            # print('obs_rot:', ee_eul)
            # print('obs_ft:', ft_sensor)
            # print('obs_joint:', real_current_joint)
            ft_data = pd.DataFrame(ft_sensor)
            #ft_data.to_excel("/home/wfh/PiH-RL/sonser_data/ft_data.xlsx",index=False)
            # time.sleep(0.1)
            # 关节角
            # current_joint = self.arm.get_current_joint_values()
            # print("当前位置：",ee_pos,"当前姿态：",ee_eul)
            self.save_data(ee_pos, ee_eul, ft_sensor, real_current_joint)
            if self._normalize_obs is True:
                ee_pos = (ee_pos - self.ee_pos_mean) * self.ee_pos_norm_scale + self.ee_pos_norm_mean
                ee_eul = (ee_eul - self.ee_rot_mean) * self.ee_rot_norm_scale + self.ee_rot_norm_mean
                ft_sensor = (ft_sensor - self.ft_mean) * self.ft_norm_scale + self.ft_norm_mean
            obs = np.concatenate((ee_pos, ee_eul, ft_sensor,real_current_joint))
            # obs = np.concatenate((ee_pos, ee_eul, ft_sensor))
            return obs
        else:
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
            ee_pos = np.copy(ee_pos)
            ee_rot = np.copy(ee_rot)
            if self._normalize_obs is True:
                ee_pos = (ee_pos - self.ee_pos_mean) * self.ee_pos_norm_scale + self.ee_pos_norm_mean
                ee_rot = (ee_rot - self.ee_rot_mean) * self.ee_rot_norm_scale + self.ee_rot_norm_mean
                ft_sensor = (ft_sensor - self.ft_mean) * self.ft_norm_scale + self.ft_norm_mean
            # obs = np.concatenate((ee_pos, ee_rot, ft_sensor))
            obs = np.concatenate((ee_pos, ee_rot, ft_sensor,current_joint))
            # obs = ee_pos
            return obs
    def save_data(self, ee_pos, ee_eul, ft_sensor, real_current_joint):

        # 创建DataFrame
        self.pih_eval.append({
            'position': np.round(ee_pos, 5),
            'orientation': np.round(ee_eul, 5),
            'ft': np.round(ft_sensor, 2),
            'joint': np.round(real_current_joint, 2),
            'action': self.action_
        })
        df = pd.DataFrame(self.pih_eval)
        # file_name = f"{self.file_counter}.xlsx"
        # file_name = f"real_evalate_0817{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
        file_name = "real_evalate_six_0819_10times06.xlsx"
        file_path = os.path.join("/home/wfh/PiH-RL/randomhole/real_pih/six", file_name)
        df.to_excel(file_path, index=False)

    def data_to_excel(self,):
        # if self.file_counter:
        df = pd.DataFrame(self.pih_eval)
        # file_name = f"{self.file_counter}.xlsx"
        file_name = f"real_evalate_0817{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
        # file_name = "real_evalate_0817.xlsx"
        file_path = os.path.join("/home/wfh/PiH-RL/randomhole/real_pih", file_name)
        df.to_excel(file_path, index=False)
        print("第i轮状态信息已保存")
    # 重置
    def reset(self) -> None:
        if self.real_robot == True:
            print("---重置真实机器人至初始状态---")
            self.file_counter += 1
            # self.data_to_excel()
            self.init_joint = np.array([0.*self.d2r, 90.*self.d2r, -90.*self.d2r, 0.*self.d2r, 0.*self.d2r, 90.*self.d2r])
            # self.init_joint = np.array([0., 1.57, -1.57, 0., 0., 0.])
            self.arm.set_joint_value_target(self.init_joint)
            # 控制机械臂完成运动
            self.arm.go()
            rospy.sleep(2)
            # time.sleep(2)
            msg = Coordinate_force()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "Coordinate_force"
            msg.wrench.force.x = self.gcf.force[0]
            msg.wrench.force.y = self.gcf.force[1]
            msg.wrench.force.z = self.gcf.force[2]
            msg.wrench.torque.x = self.gcf.torque[0]
            msg.wrench.torque.y = self.gcf.torque[1]
            msg.wrench.torque.z = self.gcf.torque[2]
            self.gcf.pub.publish(msg)
            self.init_ft_sensor = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                  msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
            # time.sleep(0.01)
            # self.real_ee_init_pos = self.rtde_r.getActualTCPPose()[:3]
            # self.z_rot_offset = np.array(self.rtde_r.getActualQ())[-1] / self.d2r
            # self.real_init_z_rot = np.array(self.rtde_r.getActualQ())[-1] / self.d2r
            # self.real_z_rot_low = self.z_rot_offset + self.z_rot_low
            # self.real_z_rot_high = self.z_rot_offset + self.z_rot_high
            # for i in range(_items):
            #     ft_record[i] = self.rtde_r.getActualTCPForce()
            #     time.sleep(0.01)
            # print(self.rtde_r.getActualTCPPose()[:3])
            # self.real_init_ft_sensor = np.mean(ft_record, axis=0)
            # print("init_ft:", self.real_init_ft_sensor)
            # print("init over.")
            # self._reset = True
        else:
            self.sim.reset()
            self.control_joints_init()
            self.init_ft_sensor = self.sim.get_ft_sensor(force_site="ee_force_sensor", torque_site="ee_torque_sensor")
    # 关节角更新调整 齐次变换
    def ee_displacement_to_target_arm_angles(self, ee_displacement: np.ndarray) ->np.ndarray:
        if self.real_robot == True:
            ee_displacement = ee_displacement
            # current_joint = np.array([self.get_joint_angle(joint=self.joint_list[i]) for i in range(6)])
            # current_joint = self.arm.get_current_joint_values()
            real_current_joint = self.arm.get_current_joint_values()
            current_ee_pos = self.forward_kinematics(real_current_joint)[0]
            current_ee_eul = R.from_quat(np.array(self.forward_kinematics(real_current_joint)[1])).as_euler("xyz",degrees=True)
            #print("初始关节角：",current_joint)
            # current_ee_pos = (self.sim.forward_kinematics(current_joint))[0]
            # current_ee_qua = np.array((self.sim.forward_kinematics(current_joint))[1])
            # current_ee_eul = R.from_quat(current_ee_qua).as_euler("xyz",degrees=True)
            # current_ee_pos = self.gcf.Compute()[:3]
            # current_ee_eul = self.gcf.Compute()[3:]
            # print("当前位置：", current_ee_pos,"当前姿态：", current_ee_eul)
            target_ee_pos = current_ee_pos + ee_displacement[:3]
            target_ee_eul = current_ee_eul + ee_displacement[3:]
            # print("Next位置：", target_ee_pos,"Next姿态：", target_ee_eul)
            target_ee_rot = np.clip(target_ee_eul, self.ee_rot_low, self.ee_rot_high)
            target_ee_pos = np.clip(target_ee_pos, self.ee_pos_low, self.ee_pos_high)
            # print("target位置：", target_ee_pos,"target姿态：", target_ee_rot)
            # 为加快收敛，quat设为0,0,1,0
            target_ee_quat = R.from_euler('xyz', target_ee_rot, degrees=True).as_quat()
            # target_ee_quat = [0,0,1,0]
            target_arm_angles = self.inverse_kinematics(
            current_joint=real_current_joint,
            target_position=target_ee_pos,
            target_orientation=target_ee_quat)
            # time.sleep(0.1)
            # return target_ee_pos, target_ee_eul
            return target_arm_angles
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

