#!/bin/bash
# 【关键修复】先关闭严格模式，避免静默退出
# set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 检查ROS Noetic环境（强制打印错误，不会静默退出）
check_ros() {
  if [ -z "$ROS_DISTRO" ] || [ "$ROS_DISTRO" != "noetic" ]; then
    echo -e "${RED}=========================================${NC}"
    echo -e "${RED}ERROR: ROS Noetic environment not loaded!${NC}"
    echo -e "${RED}Please run first: source /opt/ros/noetic/setup.bash${NC}"
    echo -e "${RED}=========================================${NC}"
    exit 1
  fi
}

# 【关键修复】自动获取工作空间路径（相对路径，不写死桌面）
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
CATKIN_WS="${SCRIPT_DIR}/../../.."

# 单独编译指定包
build_pkg() {
  local PKG=$1
  echo -e "${GREEN}[BUILD] Building package: ${PKG}${NC}"
  cd "${CATKIN_WS}" || { echo -e "${RED}Workspace not found!${NC}"; exit 1; }
  catkin_make -j2 --only-pkg-with-deps "$PKG"
}

# 全量编译
build_all() {
  echo -e "${GREEN}[BUILD] Building full workspace...${NC}"
  cd "${CATKIN_WS}" || { echo -e "${RED}Workspace not found!${NC}"; exit 1; }
  catkin_make clean
  catkin_make -j2
}

# 设置脚本权限
set_perm() {
  chmod +x "${CATKIN_WS}/src/tools/scripts/*.py" 2>/dev/null
  echo -e "${YELLOW}[INFO] Script permissions set${NC}"
}

# 交互式菜单
interactive_menu() {
  echo -e "\n===== Build Options ====="
  echo "1) Build all"
  echo "2) Build rk3588_sdk"
  echo "3) Build hesai_lidar"
  echo "4) Build orbbec_camera"
  echo "5) Build handsfree_rtk"
  echo "6) Build sensor_data"
  echo "7) Build motor_control"
  echo "8) Build tools"
  read -p "Select option (1-8): " OPT

  case $OPT in
    1) build_all ;;
    2) build_pkg rk3588_sdk ;;
    3) build_pkg hesai_lidar ;;
    4) build_pkg orbbec_camera ;;
    5) build_pkg handsfree_rtk ;;
    6) build_pkg sensor_data ;;
    7) build_pkg motor_control ;;
    8) build_pkg tools ;;
    *) echo -e "${RED}Invalid option!${NC}" && exit 1 ;;
  esac
}

# 处理命令行参数
handle_args() {
  local ARG=$1
  case $ARG in
    "build"|"all") build_all ;;
    "rk3588_sdk"|"hesai_lidar"|"orbbec_camera"|"handsfree_rtk"|"sensor_data"|"motor_control"|"tools") build_pkg "$ARG" ;;
    *) interactive_menu ;;
  esac
}

# 主函数
main() {
  echo -e "${GREEN}===== RK3588 Noetic Build Script =====${NC}"
  
  # 先检查ROS，失败会打印错误
  check_ros
  set_perm

  # 处理参数
  if [ $# -ge 1 ]; then
    handle_args "$1"
  else
    interactive_menu
  fi

  echo -e "\n${GREEN}Build completed!${NC}"
  echo "Source workspace: source ${CATKIN_WS}/devel/setup.bash"
}

# 执行主函数
main "$@"