#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import argparse
import numpy as np
from sensor_msgs.msg import NavSatFix
from lawnmower_base.srv import *

class UM982Debugger:
    def __init__(self):
        rospy.init_node('um982_debugger', anonymous=True)
        self.gps_sub = rospy.Subscriber('/lawnmower/gps', NavSatFix, self.gps_callback)
        self.gps_data = None
        
        # 等待服务
        rospy.wait_for_service('/lawnmower/set_um982_baud')
        rospy.wait_for_service('/lawnmower/set_um982_covariance')
        rospy.wait_for_service('/lawnmower/get_um982_status')
        
        # 创建服务客户端
        self.set_baud = rospy.ServiceProxy('/lawnmower/set_um982_baud', SetUM982Baud)
        self.set_cov = rospy.ServiceProxy('/lawnmower/set_um982_covariance', SetUM982Covariance)
        self.get_status = rospy.ServiceProxy('/lawnmower/get_um982_status', GetUM982Status)

    def gps_callback(self, msg):
        self.gps_data = msg

    def get_gps_info(self):
        if self.gps_data is None:
            print("No GPS data received")
            return
        
        print("\n=== UM982 GPS Info ===")
        print(f"Timestamp: {self.gps_data.header.stamp}")
        print(f"Frame ID: {self.gps_data.header.frame_id}")
        
        # GPS状态
        status_str = {
            -1: "NO_FIX",
            0: "FIX",
            1: "SBAS_FIX",
            2: "GBAS_FIX"
        }.get(self.gps_data.status.status, "UNKNOWN")
        print(f"Status: {status_str} (code: {self.gps_data.status.status})")
        
        # 经纬度高度
        print(f"\nPosition:")
        print(f"  Latitude: {self.gps_data.latitude:.8f} °")
        print(f"  Longitude: {self.gps_data.longitude:.8f} °")
        print(f"  Altitude: {self.gps_data.altitude:.2f} m")
        
        # 协方差矩阵
        print("\nPosition Covariance:")
        cov_matrix = np.array(self.gps_data.position_covariance).reshape(3,3)
        print(cov_matrix)
        print(f"Covariance type: {self.gps_data.position_covariance_type}")

    def set_baud_cmd(self, baud):
        try:
            resp = self.set_baud(baud)
            print(f"Set baud rate result: {resp.success}")
            print(f"Message: {resp.message}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def set_cov_cmd(self, cov):
        try:
            resp = self.set_cov(cov)
            print(f"Set covariance result: {resp.success}")
            print(f"Message: {resp.message}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def get_status_cmd(self):
        try:
            resp = self.get_status()
            print(f"\nUM982 Status: {'Ready' if resp.ready else 'Not Ready'}")
            print(f"Error code: {resp.error_code}")
            print(f"Error message: {resp.error_msg}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='UM982 RTK Debug Tool')
    parser.add_argument('--info', action='store_true', help='Show GPS info')
    parser.add_argument('--status', action='store_true', help='Get UM982 status')
    parser.add_argument('--set-baud', type=int, help='Set baud rate (9600/19200/115200)')
    parser.add_argument('--rate', type=int, default=1, help='Update rate (Hz)')
    
    args = parser.parse_args()
    
    debugger = UM982Debugger()
    rate = rospy.Rate(args.rate)
    
    while not rospy.is_shutdown():
        if args.info:
            debugger.get_gps_info()
        if args.status:
            debugger.get_status_cmd()
        if args.set_baud:
            debugger.set_baud_cmd(args.set_baud)
            args.set_baud = None  # 只执行一次
        
        rate.sleep()