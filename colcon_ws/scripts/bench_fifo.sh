#!/bin/bash
# ディスクを FIFO で都度生成・消滅させる案の検証。
#
#   ./scripts/bench_fifo.sh [physics_hz]
#
# 前半: ロボット 2 台・積み荷 0 枚で、場に 24 枚を保つ FIFO を回す。
#       スポーンと削除の往復時間、フレームの引っかかりを見る。
# 後半: 「一人 3 枚 x 8 人」を想定しているので、そもそもロボット 8 台が
#       積み荷 0 枚でも実時間に収まるのかを、台数を増やしながら見る。
#
# ros2_control は起動しない。物理とスポーン/削除の応答だけを見る。

HZ=${1:-200}
SIM_DIR=${SIM_DIR:-$HOME/Unity_ROS2_Robot_Simulator_v1.0.5_Linux_amd64}
WS=${WS:-$HOME/colcon_ws}
CFG=/tmp/sim_resources_fifo_${HZ}.json
LOG=/tmp/fifobench_${HZ}

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

# sample_robot_N_spawn_for_core.launch.py と同じ開始位置 (1..8)
POS_X=(-4.5  4.5 -5.5  5.5 -4.5  4.5 -5.5  5.5)
POS_Y=(-9.75 9.75 -11.25 11.25 -11.25 11.25 -9.75 9.75)
POS_YAW=(1.57 -1.57 1.57 -1.57 1.57 -1.57 1.57 -1.57)

teardown() {
  pkill -f 'bring_up_core_stage'        2>/dev/null
  pkill -f 'default_server_endpoint'    2>/dev/null
  pkill -f 'rosbridge_websocket'        2>/dev/null
  pkill -f 'Unity_ROS2_Robot_Simulator' 2>/dev/null
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

start_sim() {
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
  wait_for service /get_entities 120 || return 1
  sleep 20
  ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start > /dev/null 2>&1
  sleep 5
}

spawn_robot() {  # spawn_robot <n>
  local n=$1 i=$(($1 - 1))
  ros2 run simulation_ros2_utils spawn_entity --ros-args \
    -p urdf_path:=/tmp/bench_robot_${n}.urdf \
    -p robot_name:=sample_robot_${n} \
    -p x:="${POS_X[$i]}" -p y:="${POS_Y[$i]}" -p z:=0.15 \
    -p R:=0.0 -p P:=0.0 -p Y:="${POS_YAW[$i]}" > /dev/null 2>&1
}

python3 - <<'PY'
import os
import xacro
from ament_index_python.packages import get_package_share_directory

d = get_package_share_directory('sample_robot_description')
for n in range(1, 9):
    doc = xacro.process_file(os.path.join(d, 'robots', f'sample_robot_{n}.urdf.xacro'),
                             mappings={'use_sim': 'true'})
    with open(f'/tmp/bench_robot_{n}.urdf', 'w') as f:
        f.write(doc.toprettyxml(indent='  '))

d = get_package_share_directory('flying_disc_description')
doc = xacro.process_file(os.path.join(d, 'urdf', 'flying_disc.urdf.xacro'))
with open('/tmp/bench_disc.urdf', 'w') as f:
    f.write(doc.toprettyxml(indent='  '))
PY

############ 前半: FIFO ############
echo "############ FIFO 24 枚 (physics_hz=${HZ}, ロボット2台, 積み荷0枚) ############"
start_sim || exit 1
for n in 1 2; do spawn_robot $n; done
sleep 20

python3 "${WS}/scripts/measure_fifo.py" \
  --alive 24 --cycles 60 --interval 1.0 --prefix fifo 2>/dev/null | grep -vE "^$"

echo
echo "===== 場に 24 枚ある定常状態のフレームレート ====="
python3 "${WS}/scripts/measure_rate.py" \
  --robots sample_robot_1 sample_robot_2 --duration 20 --settle 5 \
  2>/dev/null | grep -vE "^$"
top -b -n 2 -d 2 | grep -E "Unity_R" | tail -1

############ 後半: 台数 ############
echo
echo "############ 台数と実時間性 (physics_hz=${HZ}, 積み荷0枚) ############"
start_sim || exit 1
ROBOT_LIST=""
for n in 1 2 3 4 5 6 7 8; do
  spawn_robot $n
  ROBOT_LIST="${ROBOT_LIST}sample_robot_${n} "
  if [ $((n % 2)) -eq 0 ]; then
    sleep 20
    echo
    echo "===== ロボット ${n} 台 (積み荷0枚) ====="
    # shellcheck disable=SC2086
    python3 "${WS}/scripts/measure_rate.py" --robots ${ROBOT_LIST} \
      --duration 20 --settle 5 2>/dev/null | grep -E "RTF|/clock|sample_robot_1/|測定不能"
    top -b -n 2 -d 2 | grep -E "Unity_R" | tail -1
  fi
done

echo
echo "############ 終わり ############"
teardown
