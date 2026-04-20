#!/usr/bin/env python3
import rospy
import argparse
from camera_tool import CameraTool
from lidar_tool import LiDARTool
from rtk_tool import RTKTool
class UnifiedDebugTool:
    def __init__(self):
        rospy.init_node('unified_debug_tool', anonymous=True)
        
        # 初始化所有传感器工具
        self.camera_tool = CameraTool()
        self.lidar_tool = LiDARTool()
        self.rtk_tool = RTKTool()
        
        # 解析命令行参数
        self.parser = argparse.ArgumentParser(description='Lawnmower Unified Debug Tool')
        self.subparsers = self.parser.add_subparsers(dest='command')
        
        # 添加子命令
        self.add_camera_commands()
        self.add_lidar_commands()
        self.add_rtk_commands()
        self.add_system_commands()
    
    def add_camera_commands(self):
        """添加相机相关命令"""
        camera_parser = self.subparsers.add_parser('camera', help='Camera commands')
        camera_subparsers = camera_parser.add_subparsers(dest='camera_cmd')
        
        # 相机参数设置命令
        camera_subparsers.add_parser('set_params', help='Set camera parameters')
        
        # 相机状态查询命令
        camera_subparsers.add_parser('get_status', help='Get camera status')
        
        # 相机校准命令
        camera_subparsers.add_parser('calibrate', help='Run camera calibration')
    
    def add_lidar_commands(self):
        """添加激光雷达相关命令"""
        lidar_parser = self.subparsers.add_parser('lidar', help='LiDAR commands')
        lidar_subparsers = lidar_parser.add_subparsers(dest='lidar_cmd')
        
        # 激光雷达参数设置命令
        lidar_subparsers.add_parser('set_params', help='Set LiDAR parameters')
        
        # 激光雷达状态查询命令
        lidar_subparsers.add_parser('get_status', help='Get LiDAR status')
        
        # 点云录制命令
        lidar_subparsers.add_parser('record_pcl', help='Record point cloud data')
    
    def add_rtk_commands(self):
        """添加RTK相关命令"""
        rtk_parser = self.subparsers.add_parser('rtk', help='RTK commands')
        rtk_subparsers = rtk_parser.add_subparsers(dest='rtk_cmd')
        
        # RTK参数设置命令
        rtk_subparsers.add_parser('set_params', help='Set RTK parameters')
        
        # RTK状态查询命令
        rtk_subparsers.add_parser('get_status', help='Get RTK status')
        
        # NTRIP配置命令
        rtk_subparsers.add_parser('config_ntrip', help='Configure NTRIP client')
    
    def add_system_commands(self):
        """添加系统相关命令"""
        system_parser = self.subparsers.add_parser('system', help='System commands')
        system_subparsers = system_parser.add_subparsers(dest='system_cmd')
        
        # 启动所有传感器命令
        system_subparsers.add_parser('start_all', help='Start all sensors')
        
        # 停止所有传感器命令
        system_subparsers.add_parser('stop_all', help='Stop all sensors')
        
        # 系统状态查询命令
        system_subparsers.add_parser('status', help='Get system status')
    
    def run(self):
        """运行统一调试工具"""
        args = self.parser.parse_args()
        
        if args.command == 'camera':
            if args.camera_cmd == 'set_params':
                # 实现相机参数设置逻辑
                pass
            elif args.camera_cmd == 'get_status':
                status = self.camera_tool.get_status()
                print(f"Camera Status: {status}")
        
        elif args.command == 'lidar':
            if args.lidar_cmd == 'set_params':
                # 实现激光雷达参数设置逻辑
                pass
            elif args.lidar_cmd == 'get_status':
                status = self.lidar_tool.get_status()
                print(f"LiDAR Status: {status}")
        
        elif args.command == 'rtk':
            if args.rtk_cmd == 'set_params':
                # 实现RTK参数设置逻辑
                pass
            elif args.rtk_cmd == 'get_status':
                status = self.rtk_tool.get_status()
                print(f"RTK Status: {status}")
        
        elif args.command == 'system':
            if args.system_cmd == 'start_all':
                # 实现启动所有传感器逻辑
                pass
            elif args.system_cmd == 'stop_all':
                # 实现停止所有传感器逻辑
                pass
            elif args.system_cmd == 'status':
                # 实现系统状态查询逻辑
                pass
if __name__ == '__main__':
    try:
        tool = UnifiedDebugTool()
        tool.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Unified debug tool interrupted!")