'''
Name: 
Date: 2024-05-24 22:08:57
Creator: 王飞鸿
Description: 
'''
import numpy as np
from gym_envs.envs.real_core2usb import RobotTaskEnv
from gym_envs.envs.robots.real_tx90 import TX_90
from gym_envs.envs.tasks.real_task_pihv1 import PeginHole
from gym_envs.mujoco_tx_usb import Mujoco_Func

class TX2USB2Real(RobotTaskEnv):
    metadata = {'render.modes': ['human', 'rgb_array']}
    def __init__(self,
        render: bool=False,
        render_mode: str ="human",
        normalizeObs: bool = True,
        real_robot: bool = True,
        ) -> None:
        sim = Mujoco_Func(render=render,real_robot=real_robot)
        robot = TX_90(sim=sim,_normalize=normalizeObs,real_robot=real_robot)
        task = PeginHole(sim=sim,_normalize=normalizeObs,real_robot=real_robot)
        super().__init__(robot, task, render=render)
        self.render_mode = render_mode
print('已调用tx_pih')