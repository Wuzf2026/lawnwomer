# RK3588 (ARM64) 编译工具链配置
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
# RK3588 Ubuntu20.04默认gcc/g++（arm64架构）
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
# 编译选项（适配RK3588，关闭CUDA相关）
add_definitions(-DNO_CUDA -DROS1_NOETIC -DRK3588_PLATFORM)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O2 -Wall")
# 链接选项
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lpthread -lserial -lobsensor")