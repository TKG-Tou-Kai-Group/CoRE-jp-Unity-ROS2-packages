#!/bin/bash
# コンテナイメージをビルドする。
#
#   ./build_docker_image.sh [humble|jazzy]
#
# 既定は jazzy。イメージ名は <user>/core-unity-sim-<distro> です。
# ホストと同じ uid/gid のユーザを作るので、マウントした colcon_ws の
# パーミッションがずれません。

set -e

file_dir=$(dirname "$0")

distro=${1:-jazzy}
case "${distro}" in
  humble) codename=jammy ;;
  jazzy)  codename=noble ;;
  *)
    echo "Unsupported ROS distro '${distro}'. Use humble or jazzy." >&2
    exit 1
    ;;
esac

user=$(id -un)
uid=$(id -u)
gid=$(id -g)

# 使うシミュレータのバージョン。定義はこのファイルではなく
# colcon_ws/src/sample_robot_sim/config/simulator_version.txt にある。
# launch も同じファイルを読むので、両者がずれない。
version_file="${file_dir}/../colcon_ws/src/sample_robot_sim/config/simulator_version.txt"
if [ ! -f "${version_file}" ]; then
  echo "バージョンの定義が見つかりません: ${version_file}" >&2
  exit 1
fi
simulator_version=$(grep -v '^[[:space:]]*#' "${version_file}" | grep -v '^[[:space:]]*$' | head -1 | tr -d '[:space:]')
if [ -z "${simulator_version}" ]; then
  echo "${version_file} にバージョンが書かれていません" >&2
  exit 1
fi

echo "Building ${user}/core-unity-sim-${distro} (ROS 2 ${distro} / Ubuntu ${codename})"
echo "Simulator ${simulator_version} (from $(basename "${version_file}"))"

docker build -t "${user}/core-unity-sim-${distro}" \
    --build-arg SIMULATOR_VERSION="${simulator_version}" \
    --build-arg ROS_DISTRO_ARG="${distro}" \
    --build-arg UBUNTU_CODENAME="${codename}" \
    --build-arg USER=core \
    --build-arg GROUP=core \
    --build-arg UID="${uid}" \
    --build-arg GID="${gid}" \
    "${file_dir}"
