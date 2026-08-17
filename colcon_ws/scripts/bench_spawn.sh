#!/bin/bash
# 射出のたびにディスクを生成する方式が成立するかを測る。
#
#   ./scripts/bench_spawn.sh [physics_hz] [robots]
#
# 積み荷 0 枚 (= 都度生成にしたときの状態) でロボットを置き、
# そこへ 1 枚ずつスポーンしながら遅延とフレームの引っかかりを見る。
# ros2_control は起動しない。物理とスポーンの応答だけを見たいため。

HZ=${1:-200}
ROBOTS=${2:-2}
SIM_DIR=${SIM_DIR:-$HOME/Unity_ROS2_Robot_Simulator_v1.0.5_Linux_amd64}
WS=${WS:-$HOME/colcon_ws}
CFG=/tmp/sim_resources_spawn_${HZ}.json
LOG=/tmp/spawnbench_${HZ}

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

POS_X=(-4.5  4.5 -5.5)
POS_Y=(-9.75 9.75 -11.25)
POS_YAW=(1.57 -1.57 1.57)

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

teardown

cat > "${CFG}" <<EOF
{"settings": {"physics_hz": ${HZ}},
 "spawnable_paths": [], "world_paths": [], "named_poses": []}
EOF

python3 - <<'PY'
import os
import xacro
from ament_index_python.packages import get_package_share_directory

for n in (1, 2, 3):
    d = get_package_share_directory('sample_robot_description')
    doc = xacro.process_file(os.path.join(d, 'robots', f'sample_robot_{n}.urdf.xacro'),
                             mappings={'use_sim': 'true'})
    with open(f'/tmp/bench_robot_{n}.urdf', 'w') as f:
        f.write(doc.toprettyxml(indent='  '))

d = get_package_share_directory('flying_disc_description')
doc = xacro.process_file(os.path.join(d, 'urdf', 'flying_disc.urdf.xacro'))
with open('/tmp/bench_disc.urdf', 'w') as f:
    f.write(doc.toprettyxml(indent='  '))
PY

SIMULATION_RESOURCES_CONFIG="${CFG}" \
  "${SIM_DIR}/Unity_ROS2_Robot_Simulator.x86_64" > "${LOG}_sim.log" 2>&1 &
sleep 15

ros2 launch sample_robot_sim bring_up_core_stage.launch.py \
  launch_simulator:=false > "${LOG}_stage.log" 2>&1 &
wait_for service /get_entities 120 || exit 1
sleep 20
ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start > /dev/null 2>&1
sleep 5

for n in $(seq 1 "${ROBOTS}"); do
  i=$((n - 1))
  ros2 run simulation_ros2_utils spawn_entity --ros-args \
    -p urdf_path:=/tmp/bench_robot_${n}.urdf \
    -p robot_name:=sample_robot_${n} \
    -p x:="${POS_X[$i]}" -p y:="${POS_Y[$i]}" -p z:=0.15 \
    -p R:=0.0 -p P:=0.0 -p Y:="${POS_YAW[$i]}" > /dev/null 2>&1
done
sleep 20

echo "############ 都度スポーンの成立性 (physics_hz=${HZ}, ロボット${ROBOTS}台, 積み荷0枚) ############"
echo
echo "===== 射出間隔を想定した 1.0 s 間隔 ====="
python3 "${WS}/scripts/measure_spawn.py" --count 20 --interval 1.0 --prefix slow 2>/dev/null | grep -vE "^$"

echo
echo "===== 連射を想定した 0.2 s 間隔 ====="
python3 "${WS}/scripts/measure_spawn.py" --count 20 --interval 0.2 --baseline 10 --prefix burst 2>/dev/null | grep -vE "^$"

echo
echo "===== スポーン 40 枚ぶん置いた後のフレームレート ====="
python3 "${WS}/scripts/measure_rate.py" \
  --robots $(for n in $(seq 1 "${ROBOTS}"); do echo -n "sample_robot_${n} "; done) \
  --duration 20 --settle 5 2>/dev/null | grep -vE "^$"
top -b -n 2 -d 2 | grep -E "Unity_R" | tail -1

echo
echo "############ 終わり ############"
teardown
