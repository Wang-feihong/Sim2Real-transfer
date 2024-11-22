'''
Name: 
Date: 2024-04-18 14:53:14
Creator: 王飞鸿
Description: 
'''
import numpy as np
from gym_envs.envs.core2usb import RobotTaskEnv
from gym_envs.envs.robots.tx90v2 import TX_90
from gym_envs.envs.tasks.task_pihv1 import PeginHole
from gym_tx90.gym_envs.mujoco_tx_usb_s2r import Mujoco_Func

class TX2USBEnv(RobotTaskEnv):
    metadata = {'render.modes': ['human', 'rgb_array']}
    def __init__(self,
        render: bool=False,
        render_mode: str ="human",
        normalizeObs: bool = True,
        domain_random: bool = False,
        ) -> None:
        sim = Mujoco_Func(render=render,domain_random=domain_random)
        robot = TX_90(sim=sim,_normalize=normalizeObs,)
        task = PeginHole(sim=sim,_normalize=normalizeObs)
        super().__init__(robot, task, render=render)
        self.render_mode = render_mode
print('已调用tx_pih')