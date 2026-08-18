#!/bin/bash
# 「オムニホイールをやめて、摩擦0の車体に直接力を加える」案の物理コストを測る。
#
#   ./scripts/bench_chassis.sh [physics_hz]
#
# 前半: 駆動系 (wheelN_barrel_N_link 32 個 + ハウジング 8 個) を URDF から
#       落とした車体だけのロボットを、台数を増やしながら測る。
#       ロボット 1 台は 69 リンク / 68 ジョイント / 120 コリジョンで、
#       そのうち 40 リンクが駆動系。ここが消えると何台まで載るのかを見る。
# 後半: PhysX のソルバが 2 コアぶんで頭打ちになっているので、Unity の
#       ジョブワーカー数を増やすと変わるのかを、フル装備 8 台で確かめる。
#
# 車体だけにすると base_frame_link (0.6x0.6x0.02, 20 kg) が地面に落ちて滑る。
# 摩擦0 + 直接加力の物理モデルはまさにこれなので、コストの下限として妥当。

HZ=${1:-200}
SIM_DIR=${SIM_DIR:-$HOME/Unity_ROS2_Robot_Simulator_v1.0.5_Linux_amd64}
WS=${WS:-$HOME/colcon_ws}
CFG=/tmp/sim_resources_chassis_${HZ}.json
LOG=/tmp/chassisbench_${HZ}

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

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

wait_for() {
  local end=$((SECONDS + $3))
  while [ $SECONDS -lt $end ]; do
    ros2 "$1" list 2>/dev/null | grep -qx "$2" && return 0
    sleep 2
  done
  echo "!! $2 が出ない" >&2
  return 1
}

start_sim() {  # start_sim [追加のシミュレータ引数...]
  teardown
  cat > "${CFG}" <<EOF
{"settings": {"physics_hz": ${HZ}},
 "spawnable_paths": [], "world_paths": [], "named_poses": []}
EOF
  SIMULATION_RESOURCES_CONFIG="${CFG}" \
    "${SIM_DIR}/Unity_ROS2_Robot_Simulator.x86_64" "$@" > "${LOG}_sim.log" 2>&1 &
  sleep 15
  ros2 launch sample_robot_sim bring_up_core_stage.launch.py \
    launch_simulator:=false > "${LOG}_stage.log" 2>&1 &
  wait_for service /get_entities 120 || return 1
  sleep 20
  ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start > /dev/null 2>&1
  sleep 5
}

spawn_robot() {  # spawn_robot <n> <urdf> <z>
  local n=$1 i=$(($1 - 1))
  ros2 run simulation_ros2_utils spawn_entity --ros-args \
    -p urdf_path:="$2" -p robot_name:=sample_robot_${n} \
    -p x:="${POS_X[$i]}" -p y:="${POS_Y[$i]}" -p z:="$3" \
    -p R:=0.0 -p P:=0.0 -p Y:="${POS_YAW[$i]}" > /dev/null 2>&1
}

# --- 駆動系を落とした URDF を作る -------------------------------------
python3 - <<'PY'
import os
import re
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory

d = get_package_share_directory('sample_robot_description')
DROP = re.compile(r'^wheel\d+_(barrel_\d+|housing|housing_s)_link$')

for n in range(1, 9):
    doc = xacro.process_file(os.path.join(d, 'robots', f'sample_robot_{n}.urdf.xacro'),
                             mappings={'use_sim': 'true'})
    full = f'/tmp/bench_robot_{n}.urdf'
    with open(full, 'w') as f:
        f.write(doc.toprettyxml(indent='  '))

    root = ET.parse(full).getroot()
    dropped = {l.get('name') for l in root.findall('link') if DROP.match(l.get('name') or '')}
    # 落とすリンクを子に持つジョイントも一緒に外す
    for j in list(root.findall('joint')):
        child = j.find('child')
        if child is not None and child.get('link') in dropped:
            root.remove(j)
    for l in list(root.findall('link')):
        if l.get('name') in dropped:
            root.remove(l)
    ET.ElementTree(root).write(f'/tmp/chassis_robot_{n}.urdf', encoding='unicode')
    if n == 1:
        print(f'駆動系を {len(dropped)} リンク落とした: '
              f'{len(root.findall("link"))} リンク / '
              f'{len(root.findall("joint"))} ジョイント')

doc = xacro.process_file(os.path.join(
    get_package_share_directory('flying_disc_description'), 'urdf', 'flying_disc.urdf.xacro'))
with open('/tmp/bench_disc.urdf', 'w') as f:
    f.write(doc.toprettyxml(indent='  '))
PY

############ 前半: 車体だけ ############
echo "############ 車体だけ (駆動系なし) の台数スケール physics_hz=${HZ} ############"
start_sim || exit 1
LIST=""
for n in 1 2 3 4 5 6 7 8; do
  spawn_robot $n /tmp/chassis_robot_${n}.urdf 0.10
  LIST="${LIST}sample_robot_${n} "
  if [ $((n % 2)) -eq 0 ]; then
    sleep 20
    echo
    echo "===== 車体だけ ${n} 台 ====="
    # shellcheck disable=SC2086
    python3 "${WS}/scripts/measure_rate.py" --robots ${LIST} \
      --duration 20 --settle 5 2>/dev/null | grep -E "RTF|/clock|sample_robot_1/"
    top -b -n 2 -d 2 | grep -E "Unity_R" | tail -1
  fi
done

############ 後半: ジョブワーカー数 ############
echo
echo "############ ジョブワーカー数の影響 (フル装備 8 台) ############"
for JW in 4 16; do
  start_sim -job-worker-count ${JW} || exit 1
  for n in 1 2 3 4 5 6 7 8; do spawn_robot $n /tmp/bench_robot_${n}.urdf 0.15; done
  sleep 30
  echo
  echo "===== -job-worker-count ${JW} / フル装備 8 台 ====="
  python3 "${WS}/scripts/measure_rate.py" --robots sample_robot_1 sample_robot_2 \
    --duration 20 --settle 5 2>/dev/null | grep -E "RTF|/clock|sample_robot_1/"
  top -b -n 2 -d 2 | grep -E "Unity_R" | tail -1
  nproc | sed 's/^/  コンテナから見えるコア数: /'
done

echo
echo "############ 終わり ############"
teardown
