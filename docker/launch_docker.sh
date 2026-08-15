#!/bin/bash
# コンテナを起動する。
#
#   ./launch_docker.sh [humble|jazzy]
#
# 既定は jazzy。コンテナ名は core-unity-sim-<distro> なので、humble と jazzy を
# 同時に立てられます。2 つ目以降のシェルは
#   docker exec -it core-unity-sim-jazzy /bin/bash
# で入ってください。

file_dir=$(dirname "$0")

distro=${1:-jazzy}
case "${distro}" in
  humble|jazzy) ;;
  *)
    echo "Unsupported ROS distro '${distro}'. Use humble or jazzy." >&2
    exit 1
    ;;
esac

# X の共有 (シミュレータの GUI と rviz2 を出すため)
xhost +local:root

user=$(id -un)

# nvidia-container-runtime があれば GPU を渡す。無くてもシミュレータは動くが、
# レンダリング系センサ (camera) はソフトウェア描画になって遅い。
GPU_OPT=()
if type nvidia-container-runtime >/dev/null 2>&1; then
  GPU_OPT+=(--gpus all)
fi

# WSL2 (WSLg) では GPU は /dev/dxg と /usr/lib/wsl 経由で見える。
# これを渡さないと Mesa が llvmpipe (ソフトウェア描画) に落ちて、
# カメラセンサが目に見えて遅くなる。WSL 以外では素通り。
WSL_OPT=()
if [ -e /dev/dxg ]; then
  WSL_OPT+=(--device=/dev/dxg)
  [ -d /usr/lib/wsl ] && WSL_OPT+=(-v /usr/lib/wsl:/usr/lib/wsl -e LD_LIBRARY_PATH=/usr/lib/wsl/lib)
  [ -d /mnt/wslg ]    && WSL_OPT+=(-v /mnt/wslg:/mnt/wslg -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir)
  # WSLg の GPU は /dev/dxg 経由で、コンテナには /dev/dri が現れない。
  # Mesa は DRM デバイスを見つけられないと llvmpipe (ソフトウェア描画) を選ぶので、
  # d3d12 ドライバを明示する。これが無いと GPU があっても使われない。
  WSL_OPT+=(-e GALLIUM_DRIVER=d3d12)
fi

# シミュレータは -batchmode -nographics (ヘッドレス) では起動直後に
# segfault するため、コンテナ内では必ず画面ありで動かすこと。
# ヘッドレスが要るときはコンテナの外 (ホスト) で実行する。
docker run -it --rm \
  --net=host \
  --ipc=host \
  "${GPU_OPT[@]}" \
  "${WSL_OPT[@]}" \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.Xauthority:/home/core/.Xauthority:rw" \
  -v "${file_dir}/../colcon_ws:/home/core/colcon_ws" \
  -v "${file_dir}/../tools:/home/core/tools" \
  -e XAUTHORITY=/home/core/.Xauthority \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /run/dbus/system_bus_socket:/run/dbus/system_bus_socket \
  --name "core-unity-sim-${distro}" \
  "${user}/core-unity-sim-${distro}"
