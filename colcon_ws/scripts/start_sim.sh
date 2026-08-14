#!/bin/bash
# シミュレーションを開始する (set_simulation_state: PLAYING)。
# Isaac の「左の三角ボタン」に相当。
ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start
