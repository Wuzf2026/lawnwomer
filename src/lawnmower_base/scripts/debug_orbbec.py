#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import argparse
from sensor_msgs.msg import PointCloud2
from lawnmower_base.srv import *

class OrbbecDebugger:
    def __init__(self):
        rospy.init_node('orbbec_debugger', anonymous=True)
        self.pointcloud_sub = rospy.Subscriber('/lawnmower/pointcloud', PointCloud2, self.pc_callback)
        self.pc_data = None
        
        # 等待服务
        rospy.wait_for_service('/lawnmower/set_orbbec_resolution')
        rospy.wait_for_service('/lawnmower/set_orbbec_fps')
        rospy.wait_for_service('/lawnmower/get_orbbec_status')
        
        # 创建服务客户端
        self.set_resolution = rospy.ServiceProxy('/lawnmower/set_orbbec_resolution', SetOrbbecResolution)
        self.set_fps = rospy.ServiceProxy('/lawnmower/set_orbbec_fps', SetOrbbecFPS)
        self.get_status = rospy.ServiceProxy('/lawnmower/get_orbbec_status', GetOrbbecStatus)

    def pc_callback(self, msg):
        self.pc_data = msg

    def get_pointcloud_info(self):
        if self.pc_data is None:
            print("No pointcloud data received")
            return
        
        print("\n=== Orbbec PointCloud Info ===")
        print(f"Timestamp: {self.pc_data.header.stamp}")
        print(f"Frame ID: {self.pc_data.header.frame_id}")
        print(f"Resolution: {self.pc_data.width}x{self.pc_data.height}")
        print(f"Point step: {self.pc_data.point_step} bytes")
        print(f"Row step: {self.pc_data.row_step} bytes")
        print(f"Data size: {len(self.pc_data.data)} bytes")
        print(f"Is dense: {self.pc_data.is_dense}")
        print(f"Number of points: {len(self.pc_data.data)/self.pc_data.point_step}")

    def set_resolution_cmd(self, width, height):
        try:
            resp = self.set_resolution(width, height)
            print(f"Set resolution result: {resp.success}")
            print(f"Message: {resp.message}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def set_fps_cmd(self, fps):
        try:
            resp = self.set_fps(fps)
            print(f"Set FPS result: {resp.success}")
            print(f"Message: {resp.message}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def get_status_cmd(self):
        try:
            resp = self.get_status()
            print(f"\nOrbbec Status: {'Ready' if resp.ready else 'Not Ready'}")
            print(f"Error code: {resp.error_code}")
            print(f"Error message: {resp.error_msg}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Orbbec Gemini335 Debug Tool')
    parser.add_argument('--info', action='store_true', help='Show pointcloud info')
    parser.add_argument('--status', action='store_true', help='Get Orbbec status')
    parser.add_argument('--set-res', nargs=2, type=int, help='Set resolution (width height)')
    parser.add_argument('--set-fps', type=int, help='Set FPS')
    parser.add_argument('--rate', type=int, default=1, help='Update rate (Hz)')
    
    args = parser.parse_args()
    
    debugger = OrbbecDebugger()
    rate = rospy.Rate(args.rate)
    
    while not rospy.is_shutdown():
        if args.info:
            debugger.get_pointcloud_info()
        if args.status:
            debugger.get_status_cmd()
        if args.set_res:
            debugger.set_resolution_cmd(args.set_res[0], args.set_res[1])
            args.set_res = None  # 只执行一次
        if args.set_fps:
            debugger.set_fps_cmd(args.set_fps)
            args.set_fps = None  # 只执行一次
        
        rate.sleep()