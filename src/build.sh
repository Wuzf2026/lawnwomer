#!/bin/bash
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_ros() {
  if [ -z "$ROS_DISTRO" ] || [ "$ROS_DISTRO" != "noetic" ]; then
    echo -e "${RED}错误：未加载ROS Noetic环境！${NC}"
    echo "执行：source /opt/ros/noetic/setup.bash"
    exit 1
  fi
}

build_pkg() {
  local PKG=$1
  echo -e "${GREEN}单独编译: ${PKG}${NC}"
  cd ~/Desktop/lawnmower_ws
  catkin_make -j6 --only-pkg-with-deps $PKG
}

build_all() {
  echo -e "${GREEN}全量编译...${NC}"
  cd ~/Desktop/lawnmower_ws
  catkin_make -j6
}

set_perm() {
  chmod +x ~/Desktop/lawnmower_ws/src/tools/scripts/*.py 2>/dev/null
  chmod +x ~/Desktop/lawnmower_ws/src/build.sh
}

main() {
  echo -e "${GREEN}===== RK3588 Noetic 编译脚本 =====${NC}"
  check_ros
  set_perm

  echo ""
  echo "1) 全量编译"
  echo "2) 单独编译 rk3588_sdk"
  echo "3) 单独编译 hesai_lidar"
  echo "4) 单独编译 orbbec_camera"
  echo "5) 单独编译 handsfree_rtk"
  echo "6) 单独编译 sensor_data"
  echo "7) 单独编译 motor_control"
  echo "8) 单独编译 tools"
  read -p "选择: " OPT

  case $OPT in
    1) build_all ;;
    2) build_pkg rk3588_sdk ;;
    3) build_pkg hesai_lidar ;;
    4) build_pkg orbbec_camera ;;
    5) build_pkg handsfree_rtk ;;
    6) build_pkg sensor_data ;;
    7) build_pkg motor_control ;;
    8) build_pkg tools ;;
    *) echo "无效选项" && exit 1 ;;
  esac

  echo -e "\n${GREEN}编译完成！${NC}"
  echo "执行生效：source ~/Desktop/lawnmower_ws/devel/setup.bash"
}

main