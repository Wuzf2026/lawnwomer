#!/bin/bash
set -e
# 工程路径（保持原路径不变，若工程路径不同，需修改此变量）
WSPATH=~/Desktop/lawnmower_ws
# 删除所有C++源码
find ${WSPATH}/src -name "*.cpp" -delete
find ${WSPATH}/src -name "*.h" -delete
find ${WSPATH}/src -name "*.hpp" -delete
# 删除所有Python源码（保留可执行脚本相关配置）
find ${WSPATH}/src -name "*.py" -delete
# 保留必要的配置文件和脚本
find ${WSPATH}/src -name "CMakeLists.txt" -exec touch {} \;
find ${WSPATH}/src -name "*.launch" -exec touch {} \;
find ${WSPATH}/src -name "*.yaml" -exec touch {} \;
find ${WSPATH}/src -name "*.config" -exec touch {} \;
# 保留编译脚本和命令清单到build目录
#cp ${WSPATH}/src/build.sh ${WSPATH}/src/Cmdlist.md ${WSPATH}/build/
echo "源码清理完成，已保留必要的配置文件和脚本"