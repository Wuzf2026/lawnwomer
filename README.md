# Lawnwomer 全自动割草机器人 — RK3588 完整版
- 主控：RK3588
- 雷达：禾赛 JT128
- 相机：奥比中光 Gemini335
- 定位：UM982 RTK
- 系统：ROS Noetic
- 功能：激光避障 + RTK 定位 + EKF 融合 + 全自动导航

## 一键部署
cd ~
git clone https://github.com/你的用户名/lawnwomer_ws.git
cd lawnwomer_ws
chmod +x pack.sh
./pack.sh

## 启动系统
roslaunch lawnwomer lawnwomer.launch

## 开机自启
sudo cp lawnwomer.service /etc/systemd/system/
sudo systemctl enable lawnwomer

#工程总目录结构
lawnwomer_ws/
├── .gitignore               # Git 忽略文件
├── README.md                # 项目说明
├── pack.sh                  # 一键编译脚本
├── mkimage.sh               # 工程打包脚本
├── lawnwomer.service        # 开机自启服务
└── src/
    ├── cmake/
    │   └── toolchain_rk3588.cmake    # RK3588 交叉编译工具链
    ├── lawnwomer/           # 机器人主控、导航、避障
        ├── CMakeLists.txt
        ├── package.xml
        ├── planner.xml
        ├── launch/
        │   ├── lawnwomer.launch
        │   ├── hardware.launch
        │   ├── navigation.launch
        │   └── debug.launch
        ├── config/
        │   ├── ekf.yaml
        │   ├── costmap_common_params.yaml
        │   ├── local_costmap.yaml
        │   ├── global_costmap.yaml
        │   └── planner.yaml
        ├── rviz/
        │   └── lawnwomer.rviz
        └── src/
            ├── lawnwomer_node.cpp
            └── planner/lawnwomer_planner.cpp
    ├── hesai_lidar/         # 禾赛激光雷达驱动包
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/hesai_lidar.launch
        ├── config/jt128.yaml
        └── src/hesai_lidar_node.cpp
    ├── orbbec_camera/       # 奥比中光深度相机驱动包
        ├── CMakeLists.txt
        ├── package.xml
        ├── cfg/gemini335.cfg
        ├── launch/orbbec_camera.launch
        └── src/orbbec_camera_node.cpp
    └── handsfree_rtk/       # RTK 定位驱动包
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/handsfree_rtk.launch
        └── src/handsfree_rtk_node.cpp