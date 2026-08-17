#!/bin/bash
# physics_hz は据え置きで、ロボットの台数 (= 剛体の数) だけを増やしながら
# RTF とフレームレートを測る。
#
#   ./scripts/bench_robot_load.sh [physics_hz]
#
# フレームレートが物理の「レート」ではなく「負荷」で決まっているなら、
# レートを下げなくても負荷を減らせば FPS は戻るはず。それを確かめる。
#
# ディスクを delete_entity で消す方式は、シミュレータ側で応答が返らなくなる
# ことがあったので、台数を増やす向きで測る。

HZ=${1:-200}
SIM_DIR=${SIM_DIR:-$HOME/Unity_ROS2_Robot_Simulator_v1.0.5_Linux_amd64}
WS=${WS:-$HOME/colcon_ws}
CFG=/tmp/sim_resources_robotload_${HZ}.json
LOG=/tmp/robotload_${HZ}

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

wait_for() {  # wait_for <service|topic> <name> <timeout>
  local end=$((SECONDS + $3))
  while [ $SECONDS -lt $end ]; do
    ros2 "$1" list 2>/dev/null | grep -qx "$2" && return 0
    sleep 2
  done
  echo "!! $2 が出ない" >&2
  return 1
}

measure() {  # measure <label> <robots...>
  local label=$1; shift
  echo
  echo "===== [${HZ}] ${label} ====="
  python3 "${WS}/scripts/measure_rate.py" \
    --robots "$@" --duration 20 --settle 5 2>/dev/null | grep -vE "^$"
  top -b -n 2 -d 2 | grep -E "Unity_R" | tail -1
}

echo "############ 負荷とフレームレート (physics_hz = ${HZ}) ############"
teardown

cat > "${CFG}" <<EOF
{"settings": {"physics_hz": ${HZ}},
 "spawnable_paths": [], "world_paths": [], "named_poses": []}
EOF

SIMULATION_RESOURCES_CONFIG="${CFG}" \
  "${SIM_DIR}/Unity_ROS2_Robot_Simulator.x86_64" > "${LOG}_sim.log" 2>&1 &
sleep 15

ros2 launch sample_robot_sim bring_up_core_stage.launch.py \
  launch_simulator:=false > "${LOG}_stage.log" 2>&1 &
wait_for service /get_entities 120 || exit 1
sleep 20
ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start > /dev/null 2>&1
sleep 5

measure "ロボット 0 台 (ステージのみ)" sample_robot_1

for n in 1 2 3; do
  ros2 launch sample_robot_sim sample_robot_${n}_spawn_for_core.launch.py \
    > "${LOG}_r${n}.log" 2>&1 &
  wait_for topic /sample_robot_${n}/ground_truth 180 || exit 1
  sleep 40   # ディスク 20 枚の装填と、積み荷が落ち着くまで
  measure "ロボット ${n} 台 (ディスク $((n * 20)) 枚)" \
    $(for i in $(seq 1 $n); do echo -n "sample_robot_${i} "; done)
done

echo
echo "############ 終わり ############"
teardown
