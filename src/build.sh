#!/bin/bash
set -e  

WORKSPACE_DIR=$(cd $(dirname $0)/../ && pwd)  
SRC_DIR=${WORKSPACE_DIR}/src
BUILD_DIR=${WORKSPACE_DIR}/build
DEVEL_DIR=${WORKSPACE_DIR}/devel
LIB_OUTPUT_DIR=${SRC_DIR}/libs  

source /opt/ros/noetic/setup.bash

export CMAKE_SYSTEM_PROCESSOR=aarch64
export CMAKE_BUILD_TYPE=Debug  
export CXXFLAGS="-g -O0 -march=armv8-a"  
export CFLAGS="-g -O0 -march=armv8-a"

echo "【1/5】创建编译目录和库输出目录..."
mkdir -p ${BUILD_DIR} ${DEVEL_DIR} ${LIB_OUTPUT_DIR}
mkdir -p ${LIB_OUTPUT_DIR}/rk3588_sdk
mkdir -p ${LIB_OUTPUT_DIR}/handsfree_rtk
mkdir -p ${LIB_OUTPUT_DIR}/orbbec_camera
mkdir -p ${LIB_OUTPUT_DIR}/hesai_lidar

compile_package() {
    local PKG_NAME=$1
    echo "【2/5】编译 ${PKG_NAME} 包..."
    cd ${WORKSPACE_DIR}
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
                -DCMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR} \
                --packages-select ${PKG_NAME}
}

compile_package rk3588_sdk
compile_package handsfree_rtk
compile_package orbbec_camera
compile_package hesai_lidar

echo "【3/5】提取各包库文件..."
cp -rf ${DEVEL_DIR}/lib/librk3588_sdk* ${LIB_OUTPUT_DIR}/rk3588_sdk/
cp -rf ${DEVEL_DIR}/lib/libhandsfree_rtk* ${LIB_OUTPUT_DIR}/handsfree_rtk/
cp -rf ${DEVEL_DIR}/lib/liborbbec_camera* ${LIB_OUTPUT_DIR}/orbbec_camera/
cp -rf ${DEVEL_DIR}/lib/libhesai_lidar* ${LIB_OUTPUT_DIR}/hesai_lidar/

cp -rf ${SRC_DIR}/rk3588_sdk/include/* ${LIB_OUTPUT_DIR}/rk3588_sdk/
cp -rf ${SRC_DIR}/handsfree_rtk/include/* ${LIB_OUTPUT_DIR}/handsfree_rtk/
cp -rf ${SRC_DIR}/orbbec_camera/include/* ${LIB_OUTPUT_DIR}/orbbec_camera/
cp -rf ${SRC_DIR}/hesai_lidar/include/* ${LIB_OUTPUT_DIR}/hesai_lidar/

compile_target_package() {
    local PKG_NAME=$1
    echo "【4/5】编译 ${PKG_NAME} 包..."
    cd ${WORKSPACE_DIR}
    catkin_make --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
                -DCMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR} \
                -DLIB_OUTPUT_DIR=${LIB_OUTPUT_DIR} \
                --packages-select ${PKG_NAME}
}

compile_target_package sensor_data
compile_target_package motor_control
compile_target_package tools

echo "【5/5】验证编译结果..."
CHECK_FILES=(
    "${DEVEL_DIR}/lib/sensor_data/sensor_data_node"
    "${DEVEL_DIR}/lib/motor_control/motor_control_node"
    "${DEVEL_DIR}/lib/tools/debug_tool"
)
for file in "${CHECK_FILES[@]}"; do
    if [ -f ${file} ]; then
        echo "编译成功：${file}"
    else
        echo "编译失败：${file} 不存在"
        exit 1
    fi
done

echo "所有包编译完成！可执行文件路径："
echo "sensor_data: ${DEVEL_DIR}/lib/sensor_data/sensor_data_node"
echo "motor_control: ${DEVEL_DIR}/lib/motor_control/motor_control_node"
echo "tools: ${DEVEL_DIR}/lib/tools/debug_tool"

source ${DEVEL_DIR}/setup.bash
echo "已加载ROS环境，可直接运行节点（如：rosrun sensor_data sensor_data_node）"