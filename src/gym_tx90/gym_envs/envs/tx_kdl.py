'''
Name: 
Date: 2024-01-24 22:27:08
Creator: 王飞鸿
Description: 
'''
# URDF模型的正逆运动学求解器 可以用于计算给定关节角度和目标位置姿态的末端执行器位置和姿态，以及计算给定目标位置姿态的关节角度。
import math
import numpy as np
import PyKDL as kdl

from kdl_parser.urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
# from kdl_parser.urdf_parser_py.kdl_parserTree import urdf_tree # it contains the kdl_tree_from_urdf_model
from scipy.spatial.transform import Rotation as R
from gym_envs.utils import euler_to_quaternion

d2r = math.pi / 180

class TX_kdl():
    def __init__(self, DHfile: object) -> object:
        robot = URDF.from_xml_file(DHfile)
        tree = kdl_tree_from_urdf_model(robot)  # 建立树

        # self.chain = tree.getChain("base_link", "link_6")
        self.chain = tree.getChain("base_link", "wrist_3_link")  # 串联机器人链
        # print("the UR3 .urdf model has %d bodies." % tree.getNrOfSegments())
        # print("the UR3 has %d bodies we used to controlled" % chain.getNrOfSegments())
        # print("the UR3 has %d joints we controlled" % chain.getNrOfJoints())
    # forward()方法用于计算给定关节角度下的末端执行器位置和姿态 输入6个关节角
    def forward(self, qpos):
        fk = kdl.ChainFkSolverPos_recursive(self.chain)
        pos = kdl.Frame()  # 建立坐标
        q = kdl.JntArray(self.chain.getNrOfJoints())
        for i in range(self.chain.getNrOfJoints()):  # 将输入的关节角度（qpos）赋值给该数组
            q[i] = qpos[i]
        fk_flag = fk.JntToCart(q, pos)  # 将关节角度转换为末端执行器的位置和姿态 并将计算结果存储在pos中
        f_pos = np.zeros(3)
        for i in range(3):
            f_pos[i] = pos.p[i]
        # print("urdf end-effector quaternion:", kdl.Rotation(pos.M).GetQuaternion())
        return f_pos, kdl.Rotation(pos.M).GetQuaternion()  # 末端执行器的位置（f_pos）和姿态（四元数表示）
    # inverse()方法用于计算给定目标位置和姿态下的关节角度。
    def inverse(self, init_joint, goal_pose, goal_rot):
        try:
            rot = kdl.Rotation()
            rot = rot.Quaternion(goal_rot[0], goal_rot[1], goal_rot[2], goal_rot[3]) # radium x y z w
            pos = kdl.Vector(goal_pose[0], goal_pose[1], goal_pose[2])
        except ValueError:
            print("The target pos can not be transfor to IK-function.")
        target_pos = kdl.Frame(rot, pos)
        # print(target_pos)
        fk = kdl.ChainFkSolverPos_recursive(self.chain)
        #inverse kinematics
        ik_v = kdl.ChainIkSolverVel_pinv(self.chain)
        # ik = kdl.ChainIkSolverPos_NR(chain, fk, ik_v, maxiter=100, eps=math.pow(10, -9))
        try:
            joint_limits = [
                (math.radians(-180), math.radians(180)),  # Joint 1
                (math.radians(-130), math.radians(147.5)),  # Joint 2
                (math.radians(-145), math.radians(145)),  # Joint 3
                (math.radians(-270), math.radians(270)),  # Joint 4
                (math.radians(-115), math.radians(140)),  # Joint 5
                (math.radians(-270), math.radians(270))  # Joint 6
                ]
            q_min = kdl.JntArray(self.chain.getNrOfJoints())
            q_max = kdl.JntArray(self.chain.getNrOfJoints())
            for i in range(self.chain.getNrOfJoints()):
                q_min[i] = joint_limits[i][0]
                q_max[i] = joint_limits[i][1]
        except ValueError:
            print("未输入关节范围")
        ik_p_kdl = kdl.ChainIkSolverPos_NR_JL(self.chain, q_min, q_max, fk, ik_v)
        #ik_p_kdl = kdl.ChainIkSolverPos_NR(self.chain, fk, ik_v)
        q_init = kdl.JntArray(self.chain.getNrOfJoints())
        for i in range(6):
            q_init[i] = init_joint[i]
        q_out = kdl.JntArray(self.chain.getNrOfJoints())
        ik_p_kdl.CartToJnt(q_init, target_pos, q_out)
        # print("Output angles:", q_out)
        q_out_trans = np.zeros(self.chain.getNrOfJoints())
        for i in range(self.chain.getNrOfJoints()):
            q_out_trans[i] = np.array(q_out[i])
        # print(q_out_trans)
        return (q_out_trans)