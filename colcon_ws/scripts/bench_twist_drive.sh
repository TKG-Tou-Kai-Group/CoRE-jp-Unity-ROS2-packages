#!/bin/bash
# BodyTwistDrive (車体速度を力で追従させる駆動) の検証。
#
#   SIM_DIR=/home/core/sim_dev ./scripts/bench_twist_drive.sh [physics_hz]
#
# 駆動系 40 リンクを落とし、摩擦0にして BodyTwistDrive を付けた機体で:
#   1. 指令速度への追従精度 (オムニホイールの 54% と比べる)
#   2. 台数を増やしたときの RTF とフレームレート
#   3. 押し合いが成立するか (2 台をぶつけて、押された側が動くか)
# を見る。
#
# ros2_control は起動しない。cmd_vel はシミュレータが直接購読する。

HZ=${1:-200}
SIM_DIR=${SIM_DIR:-$HOME/sim_dev}
WS=${WS:-$HOME/colcon_ws}
CFG=/tmp/sim_resources_twist_${HZ}.json
LOG=/tmp/twistbench_${HZ}

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

spawn_robot() {  # spawn_robot <n> [x] [y]
  local n=$1 i=$(($1 - 1))
  local x=${2:-${POS_X[$i]}} y=${3:-${POS_Y[$i]}}
  ros2 run simulation_ros2_utils spawn_entity --ros-args \
    -p urdf_path:=/tmp/twist_robot_${n}.urdf -p robot_name:=sample_robot_${n} \
    -p x:="${x}" -p y:="${y}" -p z:=0.10 \
    -p R:=0.0 -p P:=0.0 -p Y:="${POS_YAW[$i]}" > /dev/null 2>&1
}

# --- 駆動系を落とし、摩擦0にし、body_twist_drive を足した URDF を作る ------
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
    src = f'/tmp/twist_src_{n}.urdf'
    with open(src, 'w') as f:
        f.write(doc.toprettyxml(indent='  '))

    root = ET.parse(src).getroot()
    dropped = {l.get('name') for l in root.findall('link') if DROP.match(l.get('name') or '')}
    for j in list(root.findall('joint')):
        c = j.find('child')
        if c is not None and c.get('link') in dropped:
            root.remove(j)
    for l in list(root.findall('link')):
        if l.get('name') in dropped:
            root.remove(l)

    # 床とは擦らせない。推進は BodyTwistDrive の加力だけで行う。
    for cm in root.findall('collision_material'):
        fr = cm.find('friction')
        if fr is not None and cm.get('name') in ('robot_body', 'wheel_barrel'):
            fr.set('static', '0.0')
            fr.set('dynamic', '0.0')
            fr.set('combine', 'minimum')

    # 車輪を外すと車体が 5 cm 下がって床に直付きになる。オムニ機は車輪
    # (半径 0.05, 中心 z=0.05) で浮いており、車体下面には 5 cm の隙間があった。
    # 直付きだとフィールドの低い段差に引っ掛かって走れない (実測: 平らな
    # 地面では 100% 追従するのに、ステージ上の開始位置では 156 N を掛けても
    # 速度 0)。車輪と同じ位置・同じ半径の摩擦0の球を接地脚として足して、
    # 車高と地上高を元に戻す。collision を足すだけなのでリンクは増えず、
    # ソルバのコストはほぼ変わらない。
    base_frame = next(l for l in root.findall('link')
                      if l.get('name') == 'base_frame_link')
    for sx in (0.35, -0.35):
        for sy in (0.35, -0.35):
            col = ET.SubElement(base_frame, 'collision')
            ET.SubElement(col, 'collision_material').set('name', 'robot_body')
            o = ET.SubElement(col, 'origin')
            o.set('xyz', f'{sx} {sy} -0.01')   # base_frame_link は base_link の +0.06
            o.set('rpy', '0 0 0')
            g = ET.SubElement(col, 'geometry')
            ET.SubElement(g, 'sphere').set('radius', '0.05')

    sim = root.find('simulation')
    if sim is None:
        sim = ET.SubElement(root, 'simulation')
    drive = ET.SubElement(sim, 'sensor')
    drive.set('name', 'base_frame_link')
    drive.set('type', 'body_twist_drive')
    for tag, val in (('max_linear_velocity', '3.6'),
                     ('max_angular_velocity', '6.0'),
                     ('max_linear_acceleration', '4.0'),
                     ('max_angular_acceleration', '8.0'),
                     ('command_timeout', '0.5')):
        ET.SubElement(drive, tag).text = val

    ET.ElementTree(root).write(f'/tmp/twist_robot_{n}.urdf', encoding='unicode')
    if n == 1:
        print(f'車体のみ {len(root.findall("link"))} リンク + body_twist_drive')
PY

############ 1. 追従精度 ############
echo "############ BodyTwistDrive の検証 (physics_hz=${HZ}) ############"
start_sim || exit 1
spawn_robot 1
wait_for topic /sample_robot_1/ground_truth 120 || exit 1
sleep 15

echo
echo "===== 1. 指令速度への追従 (オムニホイールは 54% だった) ====="
grep -E "body_twist_drive" "${LOG}_sim.log" | head -2
# 1 つの試験で機体が傾いたり回ったりすると後続が巻き添えになるので、
# 試験ごとに立て直して独立に測る。車体高さ z と回転量も一緒に見て、
# 転倒・意図しない旋回が起きていないかを確認する。
for t in "--vx 0.4" "--vx 1.0" "--vy 0.4" "--wz 1.0"; do
  # shellcheck disable=SC2086
  python3 "${WS}/scripts/measure_drive.py" --robot sample_robot_1 $t --duration 5 \
    2>/dev/null | grep -vE "^$"
  echo
  start_sim || exit 1
  spawn_robot 1
  wait_for topic /sample_robot_1/ground_truth 120 || exit 1
  sleep 15
done

############ 2. 押し合い ############
echo "===== 2. 押し合いが成立するか ====="
start_sim || exit 1
# 2 台を 1.2 m 離して正面から向かい合わせ、1 台だけ前進させる
spawn_robot 1 0.0 -0.6
wait_for topic /sample_robot_1/ground_truth 120 || exit 1
spawn_robot 2 0.0 0.6
wait_for topic /sample_robot_2/ground_truth 120 || exit 1
sleep 15
python3 "${WS}/scripts/measure_push.py" --pusher sample_robot_1 --target sample_robot_2 \
  --vx 1.0 --duration 8 2>/dev/null | grep -vE "^$"

############ 3. 台数 ############
echo
echo "===== 3. 台数と実時間性 ====="
start_sim || exit 1
LIST=""
for n in 1 2 3 4 5 6 7 8; do
  spawn_robot $n
  LIST="${LIST}sample_robot_${n} "
  if [ $((n % 4)) -eq 0 ]; then
    sleep 20
    echo
    echo "--- ${n} 台 ---"
    # shellcheck disable=SC2086
    python3 "${WS}/scripts/measure_rate.py" --robots ${LIST} \
      --duration 20 --settle 5 2>/dev/null | grep -E "RTF|/clock|sample_robot_1/"
    top -b -n 2 -d 2 | grep -E "Unity_R" | tail -1
  fi
done

echo
echo "############ 終わり ############"
teardown
