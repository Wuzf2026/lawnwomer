cmake_minimum_required(VERSION 3.10)
project(lawnmower_base)

## 启用catkin
find_package(catkin REQUIRED)

## catkin配置
catkin_package()

## 安装脚本
install(DIRECTORY launch urdf config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)