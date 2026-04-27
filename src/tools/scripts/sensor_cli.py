#!/usr/bin/env python3
import rospy
import subprocess
rospy.init_node("sensor_cli")
while not rospy.is_shutdown():
    print("===== 调试指令 =====")
    print("1:查看所有传感器话题")
    print("2:重启采集节点")
    print("3:查看当前参数")
    cmd=input("输入指令：")
    if cmd=="1":
        subprocess.call("rostopic list | grep sensor_data",shell=True)
    elif cmd=="2":
        subprocess.call("rosnode kill /sensor_data_collect",shell=True)
    elif cmd=="3":
        subprocess.call("rosparam list | grep sensor_data_collect",shell=True)