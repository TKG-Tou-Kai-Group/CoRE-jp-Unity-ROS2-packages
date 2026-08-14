#!/bin/bash
# シミュレーションを停止する (set_simulation_state: STOPPED)。
ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=stop
