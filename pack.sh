#!/bin/bash
set -e
cd "$(dirname "$0")"
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
echo "========================================"
echo "  Lawnwomer 编译完成！"
echo "  启动：roslaunch lawnwomer lawnwomer.launch"
echo "========================================"