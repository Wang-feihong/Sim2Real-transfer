# Environment configuration
```shell
pip install mujoco-py mujoco-python-viewer tensorboard torch pykdl numpy ...
```
python >= 3.9.18

mujoco-py >= 2.3.0

mujoco-python-viewer >= 0.1.4

tensorboard >= 2.15.1

torch >= 1.13.1+cu116

pykdl-utils >= 0.1.0

numpy >= 1.23.1


# ROS Communication
Require：

**ROS noetic** 

**moveit**

**industrial + contorl**：
```shell
sudo apt-get install ros-noetic-industrial-core ros-noetic-open-industrial-ros-controllers

sudo apt-get install ros--moveit
```
Network Settings Name IPv4: 192.168.0.100  255.255.0.0

Initialize the workspace:
```shell
catkin_make 

source ./devel/setup.bash
```
Connecting the robot:
```shell
roslaunch staubli_tx90_planning start_assembly.py.launch
```
Connect the sensor:
```shell
roslaunch optoforce_ros optoforce_node.launch

rostopic echo /optoforce_node/wrench_HEXEA239
```
Test:
```shell
rosrun gym_tx90 init_tx90_pos_rot.py 
```
