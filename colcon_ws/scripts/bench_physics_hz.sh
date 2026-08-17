#!/bin/bash
# physics_hz を振って、フレームレート・RTF・走行・射出を一括で測る。
#
#   ./scripts/bench_physics_hz.sh 200
#   ./scripts/bench_physics_hz.sh 100
#
# ロボット 2 台 + ディスク 40 枚という、実際に操縦するときと同じ負荷で測る。
# 走行は sample_robot_1、射出は sample_robot_2 で見る。走行試験で旋回させると
# 機体の向きが変わり、そのまま射出を測ると壁までの距離が条件ごとに変わって
# しまうため、別の機体に分けている。
#
# リポジトリの config/simulation_resources.json は書き換えない。
# launch_simulator:=false でシミュレータだけ自分で起動し、
# SIMULATION_RESOURCES_CONFIG に一時ファイルを渡す。

# set -u は付けないこと。ROS の setup.bash が未定義変数を参照するため落ちる。
HZ=${1:?usage: bench_physics_hz.sh <physics_hz>}
SIM_DIR=${SIM_DIR:-$HOME/Unity_ROS2_Robot_Simulator_v1.0.5_Linux_amd64}
WS=${WS:-$HOME/colcon_ws}
CFG=/tmp/sim_resources_${HZ}.json
LOG=/tmp/bench_${HZ}

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

wait_for_service() {  # wait_for_service <name> <timeout>
  local end=$((SECONDS + $2))
  while [ $SECONDS -lt $end ]; do
    ros2 service list 2>/dev/null | grep -qx "$1" && return 0
    sleep 2
  done
  echo "!! $1 が出ない" >&2
  return 1
}

wait_for_topic() {  # wait_for_topic <name> <timeout>
  local end=$((SECONDS + $2))
  while [ $SECONDS -lt $end ]; do
    ros2 topic list 2>/dev/null | grep -qx "$1" && return 0
    sleep 2
  done
  echo "!! $1 が出ない" >&2
  return 1
}

echo "############ physics_hz = ${HZ} ############"
teardown

cat > "${CFG}" <<EOF
{"settings": {"physics_hz": ${HZ}},
 "spawnable_paths": [], "world_paths": [], "named_poses": []}
EOF

# --- シミュレータ本体 -------------------------------------------------
SIMULATION_RESOURCES_CONFIG="${CFG}" \
  "${SIM_DIR}/Unity_ROS2_Robot_Simulator.x86_64" > "${LOG}_sim.log" 2>&1 &
sleep 15

# --- endpoint / ステージ読み込み / game_manager / rosbridge -----------
ros2 launch sample_robot_sim bring_up_core_stage.launch.py \
  launch_simulator:=false > "${LOG}_stage.log" 2>&1 &

wait_for_service /get_entities 120 || exit 1
sleep 20   # load_world (startup_delay 15s) がステージを入れ終わるまで

ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start \
  > "${LOG}_start.log" 2>&1
sleep 5

# --- ロボット 2 台 ---------------------------------------------------
ros2 launch sample_robot_sim sample_robot_1_spawn_for_core.launch.py \
  > "${LOG}_r1.log" 2>&1 &
wait_for_topic /sample_robot_1/ground_truth 180 || exit 1
sleep 30   # ディスク 20 枚の装填が終わるまで

ros2 launch sample_robot_sim sample_robot_2_spawn_for_core.launch.py \
  > "${LOG}_r2.log" 2>&1 &
wait_for_topic /sample_robot_2/ground_truth 180 || exit 1
sleep 40   # 2 台目のディスクと、積み荷が落ち着くまで

echo
echo "===== [${HZ}] フレームレートと RTF (ロボット2台/ディスク40枚) ====="
python3 "${WS}/scripts/measure_rate.py" \
  --robots sample_robot_1 sample_robot_2 --duration 20 --settle 5

echo
echo "===== [${HZ}] CPU ====="
top -b -n 2 -d 3 | grep -E "Unity_R|default\+|PID" | tail -4

echo
echo "===== [${HZ}] 走行 (sample_robot_1) ====="
for t in "--vx 0.4" "--vy 0.4" "--wz 1.0"; do
  # shellcheck disable=SC2086
  python3 "${WS}/scripts/measure_drive.py" --robot sample_robot_1 $t --duration 5
  echo
done

echo "===== [${HZ}] 射出 (sample_robot_2) ====="
python3 "${WS}/scripts/measure_shoot.py" --robot sample_robot_2 --duration 90

echo
echo "############ physics_hz = ${HZ} 終わり ############"
teardown
