#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达调试工具（整合data_monitor.py/sensor_cli.py功能）
适配Hesai JT128激光雷达
"""
import argparse
import subprocess
import sys
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2

class LidarTool:
    def __init__(self):
        self.lidar_topic = "/hesai_jt128/pointcloud_raw"
        self.target_topic = "/lawnmower/pointcloud"
        self.lidar_name = "Hesai JT128"

    def get_status(self):
        """获取激光雷达运行状态"""
        rospy.init_node("lidar_tool", anonymous=True)
        print(f"[INFO] Checking {self.lidar_name} status...")
        
        # 检查话题是否发布
        topics = rospy.get_published_topics()
        raw_published = any(t[0] == self.lidar_topic for t in topics)
        target_published = any(t[0] == self.target_topic for t in topics)
        
        print(f"  - Raw pointcloud topic: {self.lidar_topic} | Published: {raw_published}")
        print(f"  - Processed topic: {self.target_topic} | Published: {target_published}")
        
        # 检查话题频率
        if raw_published:
            try:
                result = subprocess.check_output(
                    ["rostopic", "hz", "-r", "1", "-n", "5", self.lidar_topic],
                    stderr=subprocess.STDOUT,
                    encoding="utf-8"
                )
                lines = result.split("\n")
                hz_line = [l for l in lines if "average rate" in l]
                if hz_line:
                    print(f"  - Average frequency: {hz_line[0].split(':')[1].strip()}")
            except subprocess.CalledProcessError as e:
                print(f"  - Failed to get frequency: {e.output[:100]}...")

    def set_params(self, frame_id=None, rate=None):
        """设置激光雷达参数（示例）"""
        rospy.init_node("lidar_tool", anonymous=True)
        print(f"[INFO] Setting {self.lidar_name} parameters...")
        
        params = {}
        if frame_id is not None:
            params["/hesai_lidar/frame_id"] = frame_id
        if rate is not None:
            params["/hesai_lidar/publish_rate"] = rate
        
        for param, value in params.items():
            try:
                rospy.set_param(param, value)
                print(f"  - Set {param} = {value}")
            except rospy.ROSException as e:
                print(f"  - Failed to set {param}: {e}")

    def record_pcl(self, duration=10, output_bag="lidar_data.bag"):
        """录制点云数据"""
        rospy.init_node("lidar_tool", anonymous=True)
        print(f"[INFO] Recording {self.lidar_name} pointcloud for {duration}s...")
        print(f"  - Topic: {self.lidar_topic}")
        print(f"  - Output: {output_bag}")
        
        # 检查话题是否可用
        if not any(t[0] == self.lidar_topic for t in rospy.get_published_topics()):
            print(f"[ERROR] Topic {self.lidar_topic} not published!")
            return
        
        # 录制bag
        bag = rosbag.Bag(output_bag, 'w')
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(duration)
        
        def callback(msg):
            if rospy.Time.now() < end_time:
                bag.write(self.lidar_topic, msg)
        
        subscriber = rospy.Subscriber(self.lidar_topic, PointCloud2, callback)
        
        # 等待录制完成
        while rospy.Time.now() < end_time:
            rospy.sleep(0.1)
        
        subscriber.unregister()
        bag.close()
        print(f"[SUCCESS] Recorded {output_bag} | Duration: {duration}s")

    def monitor_hz(self):
        """实时监控点云频率"""
        print(f"[INFO] Real-time monitoring {self.lidar_name} frequency... (Ctrl+C to stop)")
        try:
            subprocess.run(["rostopic", "hz", self.lidar_topic], check=True)
        except KeyboardInterrupt:
            print("\n[INFO] Monitoring stopped")
        except subprocess.CalledProcessError:
            print(f"[ERROR] Failed to monitor frequency")

def main():
    parser = argparse.ArgumentParser(description=f"{LidarTool().lidar_name} Debug Tool")
    subparsers = parser.add_subparsers(dest="command", help="Subcommands")

    # 获取状态
    status_parser = subparsers.add_parser("get_status", help="Get lidar running status")
    
    # 设置参数
    set_parser = subparsers.add_parser("set_params", help="Set lidar parameters")
    set_parser.add_argument("--frame-id", help="Set frame ID (e.g. lidar_link)")
    set_parser.add_argument("--rate", type=int, help="Set publish rate (Hz)")
    
    # 录制点云
    record_parser = subparsers.add_parser("record_pcl", help="Record pointcloud data")
    record_parser.add_argument("-d", "--duration", type=int, default=10, help="Recording duration (s)")
    record_parser.add_argument("-o", "--output", default="lidar_data.bag", help="Output bag file name")
    
    # 监控频率
    hz_parser = subparsers.add_parser("monitor_hz", help="Monitor pointcloud frequency")

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    tool = LidarTool()

    if args.command == "get_status":
        tool.get_status()
    elif args.command == "set_params":
        tool.set_params(args.frame_id, args.rate)
    elif args.command == "record_pcl":
        tool.record_pcl(args.duration, args.output)
    elif args.command == "monitor_hz":
        tool.monitor_hz()

if __name__ == "__main__":
    main()