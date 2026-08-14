#!/bin/bash
# シミュレータと ROS 2 をつなぐ ROS-TCP-Endpoint。
# bring_up_core_stage.launch.py が既定で起動するので、単体で使いたいとき用。
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
