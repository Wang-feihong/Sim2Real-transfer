# ROS-Industrial driver (server) for Staubli robots

## Overview

This ROS-I driver was developed in Staubli's VAL 3 language for use with 6-axis
Staubli robot manipulators.
这个ROS-I驱动是用Staubli的VAL 3语言开发的，用于6轴Staubli机器人机械手。

It is advisable to try this driver on Staubli's emulator in Staubli Robotics Suite (SRS) first.
建议首先在Staubli机器人套件(SRS)模拟器上尝试这个驱动程序。

## Requirements

* Staubli 6-axis robot manipulator
* Staubli CS8/CS9 controller
* VAL 3 version s7.7.2 or greater
  * this is very important, since this implementation uses return values of `sioGet()`
    only available from s7.7.2 onwards
* Staubli Robotics Suite 2019 (not required but strongly recommended)

## Installation

Installing the driver to a Staubli controller simply consists of transferring the
contents of the `val3` folder to the controller itself.
安装驱动程序到Staubli控制器只需要将“val3”文件夹的内容转移到控制器。

### Clone this repository

Clone branch `master` of [staubli_val3_driver](https://github.com/ros-industrial/staubli_val3_driver):

```shell
git clone https://github.com/ros-industrial/staubli_val3_driver
```

### Transfer driver to Staubli controller

There are multiple ways of transferring VAL 3 applications to the controller:

1. Copy the contents of `val3` folder onto a USB memory stick (<2GB if using CS8), 
plugging the stick into the controller and using the teach pendant to copy the folders
将“val3”文件夹的内容复制到一个USB存储器上(如果使用CS8， <2GB)，将存储器插入控制器，并使用示教器复制文件夹

2. Use the Transfer Manager in SRS to copy the contents of `val3` folder to the controller. (Home -> Controller -> Transfer Manager)
使用SRS中的传输管理器将“val3”文件夹的内容复制到控制器。(Home ->控制器->传输管理器)

3. Use an FTP software to copy the contents of `val3` folder to the controller.
使用FTP软件将“val3”文件夹中的内容复制到控制器上。

### Open the VAL 3 application with Staubli SRS

Although it is possible to edit the source files with any text editor (they are
essentially XML files), it is advisable to use Staubli Robotics Suite:
尽管可以用任何文本编辑器编辑源文件(它们本质上是XML文件)，但建议使用Staubli Robotics Suite

* Copy contents of folder `val3` into the `usrapp` folder of the Staubli cell
将文件夹“val3”的内容复制到Staubli单元的“usrapp”文件夹
* Open the `ros_server` VAL 3 appplication located inside the `ros_server` folder
打开位于“ros server”文件夹中的`ros_server`VAL 3应用程序

SRS offers autocompletion, syntax highlighting and syntax checking, amongst other
useful features, such as a Staubli controller/teach pendant emulator.
SRS提供了自动补全、语法高亮和语法检查，以及其他有用的特性，比如Staubli控制器/仿真示教器。

## Usage

### Load driver from (real or emulated) teach pendant
从(真实的或模拟的)示教器加载驱动程序

From `Main menu`:

1. Application manager --> Val3 applications
2. +Disk --> ros_server

### Configuration
配置

The TCP sockets on the CS8/CS9 controller/emulator must be configured prior to using
the driver, otherwise a runtime error will be displayed on the teach pendant and
the driver will not work.
在使用驱动程序之前，必须配置CS8/CS9控制器/模拟控制器上的TCP套接字，否则，一个运行时的错误将显示在示教器上，并且驱动程序将不能工作。

Two sockets (TCP Servers) are required.
需要两个套接字(TCP服务器)。

#### CS8

 From `Main menu`:

1. Control panel --> I/O --> Socket --> TCP Servers
2. Configure two sockets
   * Name: Feedback, Port: 11002, Timeout: -1, End of string: 13, Nagle: Off
   * Name: Motion, Port: 11000, Timeout: -1, End of string: 13, Nagle: Off

#### CS9

 From `Home`:

1. IO --> Socket --> TCP Servers --> "+"
2. Configure two sockets
   * Name: Feedback, Port: 11002, Timeout: -1, End of string: 13, Nagle: Off
   * Name: Motion, Port: 11000, Timeout: -1, End of string: 13, Nagle: Off

### Run the driver (ROS-I server)
运行驱动程序(ROS-I服务器)

Check that:
检查

1. The contents of the `val3` folder (both `ros_server` and `ros_libs` folders)
have been transferred to the Staubli controller
“val3”文件夹(“ros_server”和“ros_libs”文件夹)的内容已转移到Staubli控制器

2. The VAL 3 application `ros_server` has been loaded
已经加载了VAL 3应用程序“ros_server”

3. Both TCP Server sockets have been configured properly
两个TCP服务器套接字都已正确配置

#### CS8

Press the `Run` button, ensure that `ros_server` is highlighted,
then press `F8` (Ok).
按' Run '按钮，确保'ros_server'高亮显示，然后按' F8 ' (Ok)。

#### CS9

VAL# --> Memory --> select `ros_server` --> ▶

Notice that depending on which mode of operation is currently active, the motors
may need to be enabled manually (a message will pop up on the screen). Likewise,
the robot will only move if the `Move` button has been pressed (or is kept pressed
if in manual mode).
请注意，根据当前活动的操作模式，可能需要手动启用电机(屏幕上将弹出一条消息)。同样，只有在“Move”按钮被按下(或者在手动模式下一直按下)时，机器人才会移动。

### Run the industrial_robot_client node (ROS-I client)
运行工业机器人客户端节点(ROS-I客户端)

The `kinetic-devel` branch provides launch files (within the `staubli_val3_driver`
ROS package). Simply run:
“kinetic-devel”分支提供启动文件(在“staubli val3 driver”ROS包内)。简单地运行：

```shell
roslaunch staubli_val3_driver robot_interface_streaming.launch robot_ip:=192.168.0.254
```

## Bugs, suggestions and feature requests

Please report any bugs you may find, make any suggestions you may have and request
new features you may find useful via GitHub.
