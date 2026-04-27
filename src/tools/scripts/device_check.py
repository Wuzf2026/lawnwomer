#!/usr/bin/env python3
import os
def main():
    print("===== RK3588 硬件设备检测 =====")
    print("串口设备：")
    os.system("ls /dev/ttyUSB* 2>/dev/null")
    print("网络状态：")
    os.system("ip a | grep eth0")
    print("ROS传感器节点：")
    os.system("rosnode list | grep sensor")
if __name__=="__main__":
    main()