#!/bin/bash
# max_delta_time を振って、フレームレートと RTF の関係を測る。
#
#   SIM_DIR=/home/core/sim_dev ./scripts/bench_max_delta.sh
#
# 条件は最初に相談を受けたときと同じ「オムニホイールのフル装備 2 台 +
# ディスク 40 枚 / physics_hz 200」。駆動系を置き換えなくても、
# 1 フレームで進める物理時間の上限を下げるだけで映像が滑らかになるかを見る。
#
# target_fps は 60 にする。既定の 10 (FrameRateController のハードコード) の
# ままだと、そこで頭打ちになって max_delta_time の効果が見えないため。
# カメラ自体は update_rate 10 Hz なので、映像は 10 FPS で頭打ちになる。
# そこに届くかどうかが目的。

HZ=${HZ:-200}
SIM_DIR=${SIM_DIR:-$HOME/sim_dev}
WS=${WS:-$HOME/colcon_ws}
LOG=/tmp/maxdelta

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

teardown() {
  pkill -f 'sample_robot_._spawn_for_core' 2>/dev/null
  pkill -f 'bring_up_core_stage'           2>/dev/null
  pkill -f 'ros2_control_node'             2>/dev/null
  pkill -f 'publisher_node'                2>/dev/null
  pkill -f 'default_server_endpoint'       2>/dev/null
  pkill -f 'rosbridge_websocket'           2>/dev/null
  pkill -f 'Unity_ROS2_Robot_Simulator'    2>/dev/null
  sleep 5
}

wait_for() {
  local end=$((SECONDS + $3))
  while [ $SECONDS -lt $end ]; do
    ros2 "$1" list 2>/dev/null | grep -qx "$2" && return 0
    sleep 2
  done
  echo "!! $2 が出ない" >&2
  return 1
}

echo "############ max_delta_time の効果 (フル装備2台 + ディスク40枚, ${HZ} Hz) ############"

for MD in 0 0.1 0.05 0.02; do
  teardown
  CFG=/tmp/sim_resources_md_${MD}.json
  if [ "${MD}" = "0" ]; then
    LABEL="既定 (0.3333, 未設定)"
    cat > "${CFG}" <<EOF
{"settings": {"physics_hz": ${HZ}, "target_fps": 60},
 "spawnable_paths": [], "world_paths": [], "named_poses": []}
EOF
  else
    LABEL="max_delta_time = ${MD}"
    cat > "${CFG}" <<EOF
{"settings": {"physics_hz": ${HZ}, "target_fps": 60, "max_delta_time": ${MD}},
 "spawnable_paths": [], "world_paths": [], "named_poses": []}
EOF
  fi

  SIMULATION_RESOURCES_CONFIG="${CFG}" \
    "${SIM_DIR}/Unity_ROS2_Robot_Simulator.x86_64" > "${LOG}_${MD}_sim.log" 2>&1 &
  sleep 15

  ros2 launch sample_robot_sim bring_up_core_stage.launch.py \
    launch_simulator:=false > "${LOG}_${MD}_stage.log" 2>&1 &
  wait_for service /get_entities 120 || exit 1
  sleep 20
  ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start > /dev/null 2>&1
  sleep 5

  ros2 launch sample_robot_sim sample_robot_1_spawn_for_core.launch.py \
    > "${LOG}_${MD}_r1.log" 2>&1 &
  wait_for topic /sample_robot_1/ground_truth 180 || exit 1
  sleep 30
  ros2 launch sample_robot_sim sample_robot_2_spawn_for_core.launch.py \
    > "${LOG}_${MD}_r2.log" 2>&1 &
  wait_for topic /sample_robot_2/ground_truth 180 || exit 1
  sleep 40

  echo
  echo "===== ${LABEL} ====="
  grep -E "SimulationSettings" "${LOG}_${MD}_sim.log" | head -5
  python3 "${WS}/scripts/measure_rate.py" \
    --robots sample_robot_1 sample_robot_2 --duration 20 --settle 5 \
    2>/dev/null | grep -vE "^$"
  top -b -n 2 -d 2 | grep -E "Unity_R" | tail -1
done

echo
echo "############ 終わり ############"
teardown
