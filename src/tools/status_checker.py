#!/usr/bin/env python3
import rospy
import rosnode
import rosbag
from diagnostic_msgs.msg import DiagnosticArray
import subprocess
class StatusChecker:
    def __init__(self):
        # 订阅诊断消息
        rospy.Subscriber('/diagnostics', DiagnosticArray, self.diagnostic_callback)
        
        # 初始化状态信息
        self.nodes_status = {}
        self.sensors_status = {
            'rtk': {'status': 'unknown', 'message': ''},
            'camera': {'status': 'unknown', 'message': ''},
            'lidar': {'status': 'unknown', 'message': ''}
        }
    
    def diagnostic_callback(self, msg):
        """诊断消息回调函数"""
        for diag in msg.status:
            if 'rtk' in diag.name:
                self.sensors_status['rtk']['status'] = diag.level
                self.sensors_status['rtk']['message'] = diag.message
            elif 'camera' in diag.name:
                self.sensors_status['camera']['status'] = diag.level
                self.sensors_status['camera']['message'] = diag.message
            elif 'lidar' in diag.name:
                self.sensors_status['lidar']['status'] = diag.level
                self.sensors_status['lidar']['message'] = diag.message
    
    def check_system_status(self):
        """检查整个系统的运行状态"""
        print("\n=== Lawnmower System Status Report ===")
        print(f"System Time: {rospy.Time.now().to_sec():.1f} seconds")
        
        # 检查ROS节点状态
        print("\n1. ROS Nodes Status:")
        nodes = rosnode.get_node_names()
        required_nodes = ['/lawnmower_base', '/rtk_driver', '/camera_driver', '/lidar_driver']
        
        for node in required_nodes:
            if node in nodes:
                print(f"   {node}: RUNNING")
                self.nodes_status[node] = 'RUNNING'
            else:
                print(f"   {node}: NOT RUNNING")
                self.nodes_status[node] = 'NOT RUNNING'
        
        # 检查传感器状态
        print("\n2. Sensors Status:")
        status_levels = {0: 'OK', 1: 'WARNING', 2: 'ERROR', 3: 'STALE'}
        for sensor, status in self.sensors_status.items():
            level = status['status']
            level_name = status_levels.get(level, 'UNKNOWN')
            print(f"   {sensor}: {level_name} - {status['message']}")
        
        # 检查硬件资源使用情况
        print("\n3. System Resources:")
        self.check_cpu_usage()
        self.check_memory_usage()
        self.check_disk_usage()
    
    def check_cpu_usage(self):
        """检查CPU使用情况"""
        result = subprocess.run(['top', '-bn1', '|', 'grep', '%Cpu'], 
                               capture_output=True, text=True)
        if result.returncode == 0:
            cpu_info = result.stdout.strip()
            print(f"   CPU Usage: {cpu_info}")
    
    def check_memory_usage(self):
        """检查内存使用情况"""
        result = subprocess.run(['free', '-h'], capture_output=True, text=True)
        if result.returncode == 0:
            mem_info = result.stdout.split('\n')[1].strip()
            print(f"   Memory Usage: {mem_info}")
    
    def check_disk_usage(self):
        """检查磁盘使用情况"""
        result = subprocess.run(['df', '-h', '/'], capture_output=True, text=True)
        if result.returncode == 0:
            disk_info = result.stdout.split('\n')[1].strip()
            print(f"   Disk Usage: {disk_info}")
    
    def save_bagfile(self, filename):
        """保存ROS bag文件"""
        print(f"Saving current data to {filename}...")
        bag = rosbag.Bag(filename, 'w')
        
        # 记录一段时间的数据（10秒）
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 10:
            # 这里可以添加需要记录的话题
            pass
        
        bag.close()
        print("Bagfile saved successfully!")
if __name__ == "__main__":
    rospy.init_node('lawnmower_status_checker')
    checker = StatusChecker()
    
    # 定期检查状态
    rate = rospy.Rate(1)  # 每秒检查一次
    while not rospy.is_shutdown():
        checker.check_system_status()
        rate.sleep()