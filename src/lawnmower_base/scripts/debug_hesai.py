#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import argparse
import numpy as np
from sensor_msgs.msg import Imu
from lawnmower_base.srv import *

class HesaiDebugger:
    def __init__(self):
        rospy.init_node('hesai_debugger', anonymous=True)
        self.imu_sub = rospy.Subscriber('/lawnmower/imu', Imu, self.imu_callback)
        self.imu_data = None
        
        # 等待服务
        rospy.wait_for_service('/lawnmower/set_hesai_ip')
        rospy.wait_for_service('/lawnmower/set_hesai_covariance')
        rospy.wait_for_service('/lawnmower/get_hesai_status')
        
        # 创建服务客户端
        self.set_ip = rospy.ServiceProxy('/lawnmower/set_hesai_ip', SetHesaiIP)
        self.set_cov = rospy.ServiceProxy('/lawnmower/set_hesai_covariance', SetHesaiCovariance)
        self.get_status = rospy.ServiceProxy('/lawnmower/get_hesai_status', GetHesaiStatus)

    def imu_callback(self, msg):
        self.imu_data = msg

    def get_imu_info(self):
        if self.imu_data is None:
            print("No IMU data received")
            return
        
        print("\n=== Hesai IMU Info ===")
        print(f"Timestamp: {self.imu_data.header.stamp}")
        print(f"Frame ID: {self.imu_data.header.frame_id}")
        
        # 姿态四元数
        print("\nOrientation (Quaternion):")
        print(f"  x: {self.imu_data.orientation.x:.6f}")
        print(f"  y: {self.imu_data.orientation.y:.6f}")
        print(f"  z: {self.imu_data.orientation.z:.6f}")
        print(f"  w: {self.imu_data.orientation.w:.6f}")
        
        # 角速度
        print("\nAngular Velocity (rad/s):")
        print(f"  x: {self.imu_data.angular_velocity.x:.6f}")
        print(f"  y: {self.imu_data.angular_velocity.y:.6f}")
        print(f"  z: {self.imu_data.angular_velocity.z:.6f}")
        
        # 线加速度
        print("\nLinear Acceleration (m/s²):")
        print(f"  x: {self.imu_data.linear_acceleration.x:.6f}")
        print(f"  y: {self.imu_data.linear_acceleration.y:.6f}")
        print(f"  z: {self.imu_data.linear_acceleration.z:.6f}")
        
        # 协方差矩阵
        print("\nOrientation Covariance:")
        print(np.array(self.imu_data.orientation_covariance).reshape(3,3))

    def set_ip_cmd(self, ip):
        try:
            resp = self.set_ip(ip)
            print(f"Set IP result: {resp.success}")
            print(f"Message: {resp.message}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def set_cov_cmd(self, ori_cov, ang_cov, lin_cov):
        try:
            resp = self.set_cov(ori_cov, ang_cov, lin_cov)
            print(f"Set covariance result: {resp.success}")
            print(f"Message: {resp.message}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def get_status_cmd(self):
        try:
            resp = self.get_status()
            print(f"\nHesai Status: {'Ready' if resp.ready else 'Not Ready'}")
            print(f"Error code: {resp.error_code}")
            print(f"Error message: {resp.error_msg}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Hesai JT128 Debug Tool')
    parser.add_argument('--info', action='store_true', help='Show IMU info')
    parser.add_argument('--status', action='store_true', help='Get Hesai status')
    parser.add_argument('--set-ip', type=str, help='Set Hesai IP address')
    parser.add_argument('--rate', type=int, default=1, help='Update rate (Hz)')
    
    args = parser.parse_args()
    
    debugger = HesaiDebugger()
    rate = rospy.Rate(args.rate)
    
    while not rospy.is_shutdown():
        if args.info:
            debugger.get_imu_info()
        if args.status:
            debugger.get_status_cmd()
        if args.set_ip:
            debugger.set_ip_cmd(args.set_ip)
            args.set_ip = None  # 只执行一次
        
        rate.sleep()