'''
Name: 
Date: 2024-03-09 10:40:34
Creator: 王飞鸿
Description: 
'''
import math
import numpy as np
import PyKDL as kdl

from kdl_parser.urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
# from kdl_parser.urdf_parser_py.kdl_parserTree import urdf_tree # it contains the kdl_tree_from_urdf_model


# robot = URDF.from_xml_file("/home/wfh/桌面/my_PiH/gym_envs/models/tx90v1.urdf")
robot = URDF.from_xml_file("/home/wfh/桌面/my_PiH/gym_envs/models/ur3_robot.urdf")  # models/ur3_robot.urdf
tree = kdl_tree_from_urdf_model(robot)  # #将URDF转换为树

print(tree.getNrOfSegments())  # 树中的线段总数
# chain = tree.getChain("base_link", "wrist_3_link")
chain = tree.getChain("base_link", "ee_link")  # 截取需要的部分构成链
print(chain.getNrOfSegments())  # 链中的段总数
print(chain.getNrOfJoints())  # 链中的关节总数

#forward kinematics
fk = kdl.ChainFkSolverPos_recursive(chain)

pos = kdl.Frame()
q = kdl.JntArray(6)
for i in range(6):
    q[i] = 0
q[0] = 3.03594358
q[1] = -1.54393884
q[2] = 1.29065137
q[3] = -1.31750886
q[4] = -1.57079632
q[5] = -1.67803805
# 23.7168     9.97208      1.0109    -19.4201    -25.1327     5.60529
# 31.7856     4.45017     1.51662    -40.5243    -33.2015     38.0089
# [3.03594358, -1.54393884,  1.29065137, -1.31750886, -1.57079632, -1.67803805]
print("fk q", q)
fk_flag = fk.JntToCart(q, pos)
# print("fk_flag", fk_flag)
# print("pos", pos)
print("rotation matrix of ee", pos.M)
print("end-effector position:", pos.p)

#inverse kinematics
ik_v = kdl.ChainIkSolverVel_pinv(chain)
# NR参数: 计算反向位置了链 fk求解器 ik求解器 最大迭代次数 位置精度
ik = kdl.ChainIkSolverPos_NR(chain, fk, ik_v, maxiter=100, eps=math.pow(10, -9))

q_init = kdl.JntArray(6)
q_out = kdl.JntArray(6)
ik.CartToJnt(q_init, pos, q_out)
print("Output angles (rads):", q_out)