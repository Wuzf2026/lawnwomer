#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机调试工具（整合device_check.py/param_config.py功能）
适配Orbbec Gemini 335相机
"""
import argparse
import subprocess
import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo

class CameraTool:
    def __init__(self):
        self.camera_topic = "/lawnmower/color/image"
        self.depth_topic = "/lawnmower/depth/image"
        self.camera_info_topic = "/lawnmower/camera_info"
        self.device_name = "Orbbec Gemini 335"

    def list_devices(self):
        """枚举相机设备（调用rosrun命令）"""
        print(f"[INFO] Listing {self.device_name} devices...")
        try:
            result = subprocess.check_output(
                ["rosrun", "orbbec_camera", "list_devices_node"],
                stderr=subprocess.STDOUT,
                encoding="utf-8"
            )
            print(result)
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] Failed to list devices: {e.output}")
        except FileNotFoundError:
            print(f"[ERROR] orbbec_camera package not found!")

    def get_status(self):
        """获取相机运行状态"""
        rospy.init_node("camera_tool", anonymous=True)
        print(f"[INFO] Checking {self.device_name} status...")
        
        # 检查话题是否发布
        topics = rospy.get_published_topics()
        color_published = any(t[0] == self.camera_topic for t in topics)
        depth_published = any(t[0] == self.depth_topic for t in topics)
        
        print(f"  - Color image topic: {self.camera_topic} | Published: {color_published}")
        print(f"  - Depth image topic: {self.depth_topic} | Published: {depth_published}")
        
        # 订阅一次相机信息
        if color_published:
            try:
                msg = rospy.wait_for_message(self.camera_topic, Image, timeout=5.0)
                print(f"  - Image resolution: {msg.width}x{msg.height}")
                print(f"  - Image encoding: {msg.encoding}")
                print(f"  - Latest timestamp: {msg.header.stamp}")
            except rospy.ROSException:
                print(f"  - No image data received in 5 seconds")

    def set_params(self, exposure=None, gain=None, resolution=None):
        """设置相机参数（示例）"""
        rospy.init_node("camera_tool", anonymous=True)
        print(f"[INFO] Setting {self.device_name} parameters...")
        
        # 模拟参数设置（实际需根据orbbec_camera的参数服务器配置）
        params = {}
        if exposure is not None:
            params["/orbbec_camera/exposure"] = exposure
        if gain is not None:
            params["/orbbec_camera/gain"] = gain
        
        for param, value in params.items():
            try:
                rospy.set_param(param, value)
                print(f"  - Set {param} = {value}")
            except rospy.ROSException as e:
                print(f"  - Failed to set {param}: {e}")

    def calibrate(self):
        """相机标定（示例命令）"""
        print(f"[INFO] Starting {self.device_name} calibration...")
        print(f"[NOTE] Use ROS camera_calibration tool:")
        print(f"  rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/lawnmower/color/image camera:=/lawnmower/camera")

def main():
    parser = argparse.ArgumentParser(description=f"{CameraTool().device_name} Debug Tool")
    subparsers = parser.add_subparsers(dest="command", help="Subcommands")

    # 枚举设备
    list_parser = subparsers.add_parser("list_devices", help="List connected camera devices")
    
    # 获取状态
    status_parser = subparsers.add_parser("get_status", help="Get camera running status")
    
    # 设置参数
    set_parser = subparsers.add_parser("set_params", help="Set camera parameters")
    set_parser.add_argument("--exposure", type=int, help="Exposure value (0-1000)")
    set_parser.add_argument("--gain", type=float, help="Gain value (0.0-10.0)")
    set_parser.add_argument("--resolution", help="Resolution (e.g. 1280x720)")
    
    # 标定
    calib_parser = subparsers.add_parser("calibrate", help="Start camera calibration")

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    tool = CameraTool()

    if args.command == "list_devices":
        tool.list_devices()
    elif args.command == "get_status":
        tool.get_status()
    elif args.command == "set_params":
        tool.set_params(args.exposure, args.gain, args.resolution)
    elif args.command == "calibrate":
        tool.calibrate()

if __name__ == "__main__":
    main()