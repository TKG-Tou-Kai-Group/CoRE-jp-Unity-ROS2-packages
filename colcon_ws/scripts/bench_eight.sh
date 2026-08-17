#!/bin/bash
# 8 台構成の通し検証。本番の launch 一式で台数を増やしながら測り、
# 最後に 8 台同時射撃で負荷をかける。
#
# 見るもの:
#   - カメラのフレームレートと RTF (操縦に必要な 10 FPS を維持できるか)
#   - Unity の CPU (物理)
#   - ros_tcp_endpoint の CPU (単一 Python プロセスで GIL に張り付く見込み)
#   - カメラ配信ノードの CPU 合計
#   - 場のディスク数 (FIFO の上限 24 が守られるか)
SIM_DIR=/home/core/sim_dev; WS=/home/core/colcon_ws; LOG=/tmp/eight
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash

kill_all() {
  # launch を殺しても配下のノードは生き残る。個別に狙って確実に落とす。
  # 残ると default_server_endpoint が二重になり、ポート 10000 の奪い合いで
  # シミュレータの ROS 接続が切断と再接続を繰り返す (実際に 275 回起きた)。
  for pat in spawn_for_core bring_up_core_stage ros2_control_node publisher_node \
             flying_disc_feeder operator_stream default_server_endpoint \
             rosbridge_websocket manager_node teleop_node robot_state_publisher \
             ffmpeg spawner load_world 'Simulator.x86_64'; do
    pkill -9 -f "$pat" 2>/dev/null
  done
  sleep 5
  return 0
}
wait_for() {
  local end=$((SECONDS+$3))
  while [ $SECONDS -lt $end ]; do
    ros2 "$1" list 2>/dev/null | grep -qx "$2" && return 0; sleep 2
  done; return 1
}
cpu_of() {  # cpu_of <pgrep パターン> -> 合計 %CPU
  local total=0
  for p in $(pgrep -f "$1" 2>/dev/null); do
    c=$(ps -p "$p" -o %cpu= 2>/dev/null | tr -d ' ')
    [ -n "$c" ] && total=$(python3 -c "print(f'{$total + $c:.1f}')")
  done
  echo "$total"
}
report() {  # report <ラベル> <機体リスト>
  local label="$1"; shift
  echo
  echo "----- $label -----"
  # shellcheck disable=SC2086
  python3 $WS/scripts/measure_rate.py --robots $* --duration 20 --settle 5 \
    2>/dev/null | grep -E "RTF|/clock|sample_robot_1/"
  echo "  Unity CPU      $(cpu_of 'Simulator.x86_64') %"
  echo "  endpoint CPU   $(cpu_of default_server_endpoint) %"
  echo "  カメラ配信 CPU $(cpu_of publisher_node) %"
  echo "  場のディスク   $(timeout 30 ros2 service call /get_entities simulation_interfaces/srv/GetEntities 2>/dev/null | grep -o 'flying_disc_[0-9]*' | sort -u | wc -l) 枚"
}

kill_all
cat > /tmp/c8.json <<'J'
{"settings":{"physics_hz":200},"spawnable_paths":[],"world_paths":[],"named_poses":[]}
J
SIMULATION_RESOURCES_CONFIG=/tmp/c8.json "$SIM_DIR/Unity_ROS2_Robot_Simulator.x86_64" > ${LOG}_sim.log 2>&1 &
sleep 15
ros2 launch sample_robot_sim bring_up_core_stage.launch.py launch_simulator:=false > ${LOG}_stage.log 2>&1 &
wait_for service /get_entities 120 || exit 1
sleep 20
ros2 run simulation_ros2_utils set_sim_state --ros-args -p set_state:=start >/dev/null 2>&1
sleep 5

echo "############ 8 台構成の通し検証 (physics_hz=200) ############"
LIST=""
for n in 1 2 3 4 5 6 7 8; do
  ros2 launch sample_robot_sim sample_robot_${n}_spawn_for_core.launch.py > ${LOG}_r${n}.log 2>&1 &
  wait_for topic /sample_robot_${n}/ground_truth 180 || exit 1
  sleep 22
  LIST="${LIST}sample_robot_${n} "
  if [ $((n % 2)) -eq 0 ]; then
    report "${n} 台" $LIST
  fi
done

echo
echo "############ 8 台同時射撃 ############"
python3 /tmp/fire_all.py --robots $LIST --duration 60 2>&1 | grep -vE "^$" &
FIREPID=$!
sleep 20
report "8 台・射撃中" $LIST
wait $FIREPID 2>/dev/null
sleep 10
report "8 台・射撃後" $LIST

echo
echo "=== feeder のログ (末尾) ==="
grep -E "を装填|回収|失敗|応答なし" ${LOG}_stage.log | tail -5
kill_all
exit 0
