cmake_minimum_required(VERSION 3.10.2)
project(lawn_mower)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rk3588_sdk
    orbbec_camera
    hesai_lidar
    handsfree_rtk
    motor_control
    ultrasonic_sensor
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy rk3588_sdk orbbec_camera hesai_lidar handsfree_rtk motor_control ultrasonic_sensor
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# 安装launch和配置文件
install(DIRECTORY launch/ config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)