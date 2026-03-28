#!/bin/bash
set -e

# 全局配置
ROS_DISTRO=noetic
TOOLCHAIN_FILE=./rk3588_toolchain.cmake
WS_DIR=$(pwd)
SRC_DIR=${WS_DIR}/src

# 帮助信息
usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -a          编译整个工程"
    echo "  -p <pkg>    单独编译指定功能包（handsfree_rtk/hesai_lidar/motor_control/orbbec_camera/rk3588_sdk/ultrasonic_sensor）"
    echo "  -h          显示帮助"
}

# 初始化编译环境
init_env() {
    source /opt/ros/${ROS_DISTRO}/setup.bash
    mkdir -p ${WS_DIR}/build ${WS_DIR}/devel
}

# 整体编译
build_all() {
    init_env
    cd ${WS_DIR}
    catkin_make -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE} -DCMAKE_BUILD_TYPE=Release
    echo "整体编译完成！"
}

# 单独编译指定包
build_package() {
    local pkg=$1
    if [ ! -d "${SRC_DIR}/${pkg}" ]; then
        echo "错误：功能包 ${pkg} 不存在！"
        exit 1
    fi
    init_env
    cd ${WS_DIR}
    catkin_make --cmake-args -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE} -DCMAKE_BUILD_TYPE=Release --pkg ${pkg}
    echo "单独编译 ${pkg} 完成！"
}

# 解析参数
while getopts "ap:h" opt; do
    case $opt in
        a)
            build_all
            ;;
        p)
            build_package ${OPTARG}
            ;;
        h)
            usage
            exit 0
            ;;
        *)
            usage
            exit 1
            ;;
    esac
done

# 无参数时显示帮助
if [ $# -eq 0 ]; then
    usage
    exit 1
fi