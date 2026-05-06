#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RTK调试工具（整合param_config.py/data_monitor.py功能）
适配UM982 RTK模块
"""
import argparse
import subprocess
import sys
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class RTKTool:
    def __init__(self):
        self.ntrip_topic = "/um982/nmea_raw"
        self.fix_topic = "/lawnmower/gps/fix"
        self.rtk_name = "UM982 RTK"

    def get_status(self):
        """获取RTK运行状态"""
        rospy.init_node("rtk_tool", anonymous=True)
        print(f"[INFO] Checking {self.rtk_name} status...")
        
        # 检查话题是否发布
        topics = rospy.get_published_topics()
        nmea_published = any(t[0] == self.ntrip_topic for t in topics)
        fix_published = any(t[0] == self.fix_topic for t in topics)
        
        print(f"  - NMEA raw topic: {self.ntrip_topic} | Published: {nmea_published}")
        print(f"  - GPS fix topic: {self.fix_topic} | Published: {fix_published}")
        
        # 读取NMEA数据
        if nmea_published:
            try:
                msg = rospy.wait_for_message(self.ntrip_topic, String, timeout=5.0)
                print(f"  - Latest NMEA data: {msg.data[:100]}...")
            except rospy.ROSException:
                print(f"  - No NMEA data received in 5 seconds")
        
        # 读取GPS定位
        if fix_published:
            try:
                msg = rospy.wait_for_message(self.fix_topic, NavSatFix, timeout=5.0)
                print(f"  - Latitude: {msg.latitude:.6f}")
                print(f"  - Longitude: {msg.longitude:.6f}")
                print(f"  - Altitude: {msg.altitude:.2f}m")
                print(f"  - Fix status: {msg.status.status} (0=No fix, 1=GPS fix, 2=DGPS fix)")
            except rospy.ROSException:
                print(f"  - No GPS fix data received in 5 seconds")

    def set_params(self, baudrate=None, rate=None):
        """设置RTK参数（示例）"""
        rospy.init_node("rtk_tool", anonymous=True)
        print(f"[INFO] Setting {self.rtk_name} parameters...")
        
        params = {}
        if baudrate is not None:
            params["/handsfree_rtk/baudrate"] = baudrate
        if rate is not None:
            params["/handsfree_rtk/publish_rate"] = rate
        
        for param, value in params.items():
            try:
                rospy.set_param(param, value)
                print(f"  - Set {param} = {value}")
            except rospy.ROSException as e:
                print(f"  - Failed to set {param}: {e}")

    def config_ntrip(self, host=None, port=None, mountpoint=None, user=None, password=None):
        """配置NTRIP客户端（示例）"""
        rospy.init_node("rtk_tool", anonymous=True)
        print(f"[INFO] Configuring {self.rtk_name} NTRIP client...")
        
        ntrip_params = {}
        if host:
            ntrip_params["/handsfree_rtk/ntrip/host"] = host
        if port:
            ntrip_params["/handsfree_rtk/ntrip/port"] = port
        if mountpoint:
            ntrip_params["/handsfree_rtk/ntrip/mountpoint"] = mountpoint
        if user:
            ntrip_params["/handsfree_rtk/ntrip/user"] = user
        if password:
            ntrip_params["/handsfree_rtk/ntrip/password"] = password
        
        if not ntrip_params:
            print(f"[WARNING] No NTRIP parameters provided!")
            print(f"[EXAMPLE] --host ntrip.example.com --port 2101 --mountpoint RTCM3")
            return
        
        for param, value in ntrip_params.items():
            try:
                rospy.set_param(param, value)
                print(f"  - Set {param} = {value}")
            except rospy.ROSException as e:
                print(f"  - Failed to set {param}: {e}")
        
        print(f"[NOTE] Restart handsfree_rtk node to apply NTRIP config!")

def main():
    parser = argparse.ArgumentParser(description=f"{RTKTool().rtk_name} Debug Tool")
    subparsers = parser.add_subparsers(dest="command", help="Subcommands")

    # 获取状态
    status_parser = subparsers.add_parser("get_status", help="Get RTK running status")
    
    # 设置参数
    set_parser = subparsers.add_parser("set_params", help="Set RTK parameters")
    set_parser.add_argument("--baudrate", type=int, help="Serial baudrate (e.g. 115200)")
    set_parser.add_argument("--rate", type=int, help="Publish rate (Hz)")
    
    # 配置NTRIP
    ntrip_parser = subparsers.add_parser("config_ntrip", help="Configure NTRIP client")
    ntrip_parser.add_argument("--host", help="NTRIP caster host")
    ntrip_parser.add_argument("--port", type=int, help="NTRIP caster port")
    ntrip_parser.add_argument("--mountpoint", help="NTRIP mountpoint")
    ntrip_parser.add_argument("--user", help="NTRIP username")
    ntrip_parser.add_argument("--password", help="NTRIP password")

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    tool = RTKTool()

    if args.command == "get_status":
        tool.get_status()
    elif args.command == "set_params":
        tool.set_params(args.baudrate, args.rate)
    elif args.command == "config_ntrip":
        tool.config_ntrip(args.host, args.port, args.mountpoint, args.user, args.password)

if __name__ == "__main__":
    main()