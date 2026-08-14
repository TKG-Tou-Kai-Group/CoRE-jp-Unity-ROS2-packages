#!/bin/bash
# Unity のシミュレータ本体を起動する。
# bring_up_core_stage.launch.py は既定でこれと同じ実行ファイルを自分で起動するので、
# 手動で先に立ち上げたいとき (バージョンを差し替えたい、別ターミナルでログを見たい等) 用。
SIM_DIR=${SIM_DIR:-~/Unity_ROS2_Robot_Simulator_v1.0.2_Linux_amd64}
"${SIM_DIR}"/Unity_ROS2_Robot_Simulator.x86_64 "$@"
