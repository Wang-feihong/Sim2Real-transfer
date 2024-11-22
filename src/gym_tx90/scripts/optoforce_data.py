#!/usr/bin/env python3
#coding=utf-8
import rospy
from geometry_msgs.msg import WrenchStamped
import csv
import datetime

def callback(data):
    # 创建一个包含当前时间和所有力/扭矩值的列表
    now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 去掉毫秒的最后三位以保持精度
    # 分别获取force和torque的x, y, z值
    force_values = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
    torque_values = [data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
    # 将当前时间和力/扭矩值合并为一个列表
    values = [now] + force_values + torque_values
    # 将数据写入CSV文件
    with open('optoforce_data.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(values)
def listener():
    # 初始化节点
    rospy.init_node('optoforce_data_saver', anonymous=True)
    # 创建一个订阅者，指定话题名、消息类型、回调函数
    rospy.Subscriber("/optoforce_node/wrench_HEXEA239", WrenchStamped, callback)
    # 防止节点立即退出
    rospy.spin()
if __name__ == '__main__':
    listener()