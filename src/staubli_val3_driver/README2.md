# Staubli VAL3 driver

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![Github Issues](https://img.shields.io/github/issues/ros-industrial/staubli_val3_driver.svg)](http://github.com/ros-industrial/staubli_val3_driver/issues)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


## Overview

This repository contains the `staubli_val3_driver` package which provides a set of VAL3 libraries and an application which together implement a [simple_message][] compatible server implementation.
Together with the nodes in [industrial_robot_client][], this server can be used as a ROS 1 driver that allows motion control of Staubli CS8/CS9 controlled robots, by exposing a [FollowJointTrajectory][] [action][] server, which is compatible with MoveIt and other nodes that implement an action client.

这个存储库包含了'staubli_val3_driver'包，它提供了一组val3库和一个应用程序，一起实现了一个[simple_message][]兼容的server。该server与[industrial_robot_client][]中的节点一起作为一个ROS1驱动程序，允许Staubli CS8/CS9控制器进行机器人的运动控制，通过发布一个[FollowJointTrajectory][] [action][]server，这是兼容MoveIt和其他节点，实现一个action客户端。

## Documentation

Refer to the `staubli_val3_driver` [readme](./staubli_val3_driver/README.md) for more information on requirements, setup and use.


## Compatibility

The current version of the driver is compatible with Staubli CS8 and CS9 controllers.



[simple_message]: http://wiki.ros.org/simple_message
[industrial_robot_client]: http://wiki.ros.org/industrial_robot_client
[FollowJointTrajectory]: http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html
[action]: http://wiki.ros.org/actionlib
