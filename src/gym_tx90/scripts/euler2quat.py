'''
Name: 
Date: 2024-06-11 16:38:20
Creator: 王飞鸿
Description: 
'''
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial.transform import Rotation as R

target_ee_rot = [0,0,90]
Rx, Ry, Rz = target_ee_rot
print("Rx, Ry, Rz:",Rx, Ry, Rz)

def Rxyz2quaternion(Rx, Ry, Rz):
    pi = 3.1415926535
    (roll, pitch, yaw) = (Rz * pi / 180,
                          Ry * pi / 180,
                          Rx * pi / 180)
    (x, y, z, w) = quaternion_from_euler(roll, pitch, yaw, axes='szyx')
    return x, y, z, w
# def Euler2quat(target_ee_rot):
#     R.from_euler('xyz', target_ee_rot, degrees=True).as_quat()

print("R2q:",Rxyz2quaternion(Rx, Ry, Rz))
print('E2q:',R.from_euler('xyz', target_ee_rot, degrees=True).as_quat())
# print('欧拉转:',R.from_euler('xyz', target_ee_rot, degrees=True).as_quat())

import math
def quaternion_to_euler(w, x, y, z):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z
w, x, y, z = 0, -0.7071, 0.7071, 0
roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
print("xyz:",roll, pitch, yaw)