#!/bin/bash
# CoRE ステージをワールドとして読み込む。
# bring_up_core_stage.launch.py がやっていることの手動版。ステージだけ入れ直したいとき用。
#
# 注意: load_world はスポーン済みのエンティティを全部消して STOPPED に戻す。
set -e
WORLD=${1:-$(ros2 pkg prefix core_stage_description)/share/core_stage_description/worlds/core_stage.world}
echo "loading ${WORLD}"
ros2 run core_sim_utils load_world --ros-args -p world_path:="${WORLD}"
