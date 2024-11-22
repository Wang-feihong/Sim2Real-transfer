#!/usr/bin/env python3
#coding=utf-8
import numpy as np
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
                round(pose.pose.position.z , 5),
                round(Rx, 2),
                round(Ry, 2),
                round(Rz, 2)]
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

class tx90():
    def __init__(self) -> None:
        self.gcf = GetCoordinateForce()
        # 发布力传感器节点话题
        self.pub = rospy.Publisher('/optoforce_node/reset', Empty)
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
        current_joint = self.arm.get_current_joint_values()
        # 设置机械臂的初始位置，使用六轴的位置数据进行描述（单位：弧度）
        self.d2r = self.pi/180
        self.init_joint = np.array([-0.*self.d2r, 90.*self.d2r, -90.*self.d2r, 0.*self.d2r, 0.*self.d2r, 90.*self.d2r])
        self.arm.set_joint_value_target(self.init_joint)
        # 控制机械臂完成运动
        self.arm.go()
        rospy.sleep(2)
        print("current_joint:",current_joint)
        ee_pos = self.gcf.Compute()[:3]
        ee_eul = self.gcf.Compute()[3:]
        print("current_pos_eul:", ee_pos, ee_eul)
        # print("运动后位置_关节角计算：",self.forward_kinematics(self.arm.get_current_joint_values())[0],"运动后姿态_关节角计算：",R.from_quat(self.forward_kinematics(self.arm.get_current_joint_values())[1]).as_euler("xyz",degrees=True))


if __name__ == '__main__':
    try:
        tx90 = tx90()

    except rospy.ROSInterruptException:
        pass
