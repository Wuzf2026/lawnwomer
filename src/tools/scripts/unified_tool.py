#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一传感器调试工具
整合camera_tool/lidar_tool/rtk_tool功能
"""
import argparse
import subprocess
import sys
import rospy

class UnifiedTool:
    def __init__(self):
        self.script_dir = sys.path[0]
        self.camera_tool = f"{self.script_dir}/camera_tool.py"
        self.lidar_tool = f"{self.script_dir}/lidar_tool.py"
        self.rtk_tool = f"{self.script_dir}/rtk_tool.py"

    def system_start_all(self):
        """启动所有传感器节点"""
        print("[INFO] Starting all sensor nodes...")
        launch_commands = [
            "roslaunch lawnmower_base base.launch",
            "roslaunch lawnmower_base sensors.launch",
            "roslaunch motor_control motor_control.launch"
        ]
        
        for cmd in launch_commands:
            print(f"  - Executing: {cmd}")
            try:
                # 后台启动节点（实际使用时可调整为nohup）
                subprocess.Popen(cmd.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                rospy.sleep(2)  # 等待节点启动
            except Exception as e:
                print(f"  - Failed to start: {e}")
        
        print("[SUCCESS] All sensor nodes started (check with rosnode list)")

    def system_stop_all(self):
        """停止所有传感器节点"""
        print("[INFO] Stopping all sensor nodes...")
        try:
            # 停止所有ROS节点（保留rosout）
            nodes = subprocess.check_output(["rosnode", "list"], encoding="utf-8").split()
            for node in nodes:
                if node not in ["/rosout", "/roscore"]:
                    subprocess.run(["rosnode", "kill", node], check=True)
            print("[SUCCESS] All sensor nodes stopped")
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] Failed to stop nodes: {e}")

    def system_status(self):
        """查询系统整体状态"""
        print("[INFO] Checking system status...")
        # 检查ROS核心
        try:
            subprocess.check_output(["rosnode", "info", "/rosout"], stderr=subprocess.STDOUT)
            print("  - ROS Core: Running")
        except subprocess.CalledProcessError:
            print("  - ROS Core: Not running")
        
        # 检查节点数量
        nodes = subprocess.check_output(["rosnode", "list"], encoding="utf-8").split()
        print(f"  - Active nodes: {len(nodes)}")
        
        # 检查话题数量
        topics = subprocess.check_output(["rostopic", "list"], encoding="utf-8").split()
        lawnmower_topics = [t for t in topics if "lawnmower" in t]
        print(f"  - Lawnmower topics: {len(lawnmower_topics)}")
        
        # 调用各传感器状态检查
        print("\n[INFO] Sensor status:")
        self.run_tool("camera", "get_status")
        self.run_tool("lidar", "get_status")
        self.run_tool("rtk", "get_status")

    def run_tool(self, sensor_type, subcmd, args_list=None):
        """调用指定传感器工具"""
        tool_path = {
            "camera": self.camera_tool,
            "lidar": self.lidar_tool,
            "rtk": self.rtk_tool
        }.get(sensor_type)
        
        if not tool_path:
            print(f"[ERROR] Unknown sensor type: {sensor_type}")
            return
        
        cmd = ["python3", tool_path, subcmd]
        if args_list:
            cmd.extend(args_list)
        
        try:
            result = subprocess.check_output(cmd, stderr=subprocess.STDOUT, encoding="utf-8")
            print(f"  - {sensor_type.upper()} {subcmd}:")
            for line in result.split("\n"):
                if line.strip():
                    print(f"    {line}")
        except subprocess.CalledProcessError as e:
            print(f"  - {sensor_type.upper()} {subcmd} failed: {e.output[:100]}...")

def main():
    # 主解析器
    parser = argparse.ArgumentParser(description='Unified Debug Tool for Lawnmower')
    subparsers = parser.add_subparsers(dest='main_command', required=True, help='Main commands')

    # 1. System子命令
    system_parser = subparsers.add_parser('system', help='System control commands')
    system_subparsers = system_parser.add_subparsers(dest='system_command', required=True)
    system_start = system_subparsers.add_parser('start_all', help='Start all devices')
    system_stop = system_subparsers.add_parser('stop_all', help='Stop all devices')
    system_status = system_subparsers.add_parser('status', help='Check system status')

    # 2. Camera子命令
    camera_parser = subparsers.add_parser('camera', help='Camera related commands')
    camera_subparsers = camera_parser.add_subparsers(dest='camera_command', required=True)
    camera_status = camera_subparsers.add_parser('get_status', help='Get camera status')
    camera_set = camera_subparsers.add_parser('set_params', help='Set camera parameters')

    # 3. Lidar子命令（修复核心错误行）
    lidar_parser = subparsers.add_parser('lidar', help='Lidar related commands')
    lidar_subparsers = lidar_parser.add_subparsers(dest='lidar_command', required=True)
    lidar_status = lidar_subparsers.add_parser('get_status', help='Get lidar status')
    lidar_set = lidar_subparsers.add_parser('set_params', help='Set lidar parameters')
    lidar_record = lidar_subparsers.add_parser('record_pcl', help='Record pointcloud')
    lidar_monitor = lidar_subparsers.add_parser('monitor_hz', help='Monitor lidar hz')

    # 4. RTK子命令
    rtk_parser = subparsers.add_parser('rtk', help='RTK related commands')
    rtk_subparsers = rtk_parser.add_subparsers(dest='rtk_command', required=True)
    rtk_status = rtk_subparsers.add_parser('get_status', help='Get RTK status')
    rtk_set = rtk_subparsers.add_parser('set_params', help='Set RTK parameters')
    rtk_ntrip = rtk_subparsers.add_parser('config_ntrip', help='Config NTRIP')

    # 解析参数
    args = parser.parse_args()

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    tool = UnifiedTool()
    rospy.init_node("unified_tool", anonymous=True)

    # 系统命令
    if args.module == "system":
        if args.system_cmd == "start_all":
            tool.system_start_all()
        elif args.system_cmd == "stop_all":
            tool.system_stop_all()
        elif args.system_cmd == "status":
            tool.system_status()
    
    # 相机命令
    elif args.module == "camera":
        args_list = []
        if args.camera_cmd == "set_params":
            if args.exposure:
                args_list.extend(["--exposure", str(args.exposure)])
            if args.gain:
                args_list.extend(["--gain", str(args.gain)])
        tool.run_tool("camera", args.camera_cmd, args_list)
    
    # 激光雷达命令
    elif args.module == "lidar":
        args_list = []
        if args.lidar_cmd == "set_params":
            if args.frame_id:
                args_list.extend(["--frame-id", args.frame_id])
            if args.rate:
                args_list.extend(["--rate", str(args.rate)])
        elif args.lidar_cmd == "record_pcl":
            args_list.extend(["-d", str(args.duration)])
            if args.output:
                args_list.extend(["-o", args.output])
        tool.run_tool("lidar", args.lidar_cmd, args_list)
    
    # RTK命令
    elif args.module == "rtk":
        args_list = []
        if args.rtk_cmd == "set_params":
            if args.baudrate:
                args_list.extend(["--baudrate", str(args.baudrate)])
            if args.rate:
                args_list.extend(["--rate", str(args.rate)])
        elif args.rtk_cmd == "config_ntrip":
            if args.host:
                args_list.extend(["--host", args.host])
            if args.port:
                args_list.extend(["--port", str(args.port)])
            if args.mountpoint:
                args_list.extend(["--mountpoint", args.mountpoint])
        tool.run_tool("rtk", args.rtk_cmd, args_list)

if __name__ == "__main__":
    main()