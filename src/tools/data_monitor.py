#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
import numpy as np
class DataMonitor:
    def __init__(self):
        # 订阅传感器数据
        rospy.Subscriber('/lawnmower/pointcloud', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/lawnmower/imu', Imu, self.imu_callback)
        rospy.Subscriber('/lawnmower/gps', NavSatFix, self.gps_callback)
        
        # 初始化数据统计
        self.pointcloud_stats = {'count': 0, 'min_z': float('inf'), 'max_z': -float('inf')}
        self.imu_stats = {'count': 0, 'avg_angular_velocity': np.zeros(3), 'avg_acceleration': np.zeros(3)}
        self.gps_stats = {'count': 0, 'avg_latitude': 0, 'avg_longitude': 0, 'avg_altitude': 0}
        
        # 创建定时器定期输出统计信息
        self.timer = rospy.Timer(rospy.Duration(10), self.print_statistics)
    
    def pointcloud_callback(self, msg):
        """点云数据回调函数"""
        self.pointcloud_stats['count'] += 1
        
        # 统计点云Z轴范围（简化实现）
        if msg.data:
            # 假设点云格式为XYZ
            num_points = msg.width * msg.height
            z_values = np.frombuffer(msg.data[8::12], dtype=np.float32)  # 每12字节中的第3个float
            self.pointcloud_stats['min_z'] = min(self.pointcloud_stats['min_z'], np.min(z_values))
            self.pointcloud_stats['max_z'] = max(self.pointcloud_stats['max_z'], np.max(z_values))
    
    def imu_callback(self, msg):
        """IMU数据回调函数"""
        self.imu_stats['count'] += 1
        
        # 计算角速度平均值
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.imu_stats['avg_angular_velocity'] = (self.imu_stats['avg_angular_velocity'] * 
                                                 (self.imu_stats['count'] - 1) + angular_velocity) / self.imu_stats['count']
        
        # 计算加速度平均值
        acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.imu_stats['avg_acceleration'] = (self.imu_stats['avg_acceleration'] * 
                                             (self.imu_stats['count'] - 1) + acceleration) / self.imu_stats['count']
    
    def gps_callback(self, msg):
        """GPS数据回调函数"""
        self.gps_stats['count'] += 1
        self.gps_stats['avg_latitude'] = (self.gps_stats['avg_latitude'] * (self.gps_stats['count'] - 1) + msg.latitude) / self.gps_stats['count']
        self.gps_stats['avg_longitude'] = (self.gps_stats['avg_longitude'] * (self.gps_stats['count'] - 1) + msg.longitude) / self.gps_stats['count']
        self.gps_stats['avg_altitude'] = (self.gps_stats['avg_altitude'] * (self.gps_stats['count'] - 1) + msg.altitude) / self.gps_stats['count']
    
    def print_statistics(self, event):
        """打印统计信息"""
        print("\n=== Lawnmower Data Monitor Statistics ===")
        print(f"Time: {rospy.Time.now().to_sec():.1f} seconds")
        
        print("\n1. Point Cloud Statistics:")
        print(f"   Total points received: {self.pointcloud_stats['count']}")
        print(f"   Z-axis range: {self.pointcloud_stats['min_z']:.3f}m - {self.pointcloud_stats['max_z']:.3f}m")
        
        print("\n2. IMU Statistics:")
        print(f"   Total IMU messages: {self.imu_stats['count']}")
        print(f"   Average angular velocity: {self.imu_stats['avg_angular_velocity']} rad/s")
        print(f"   Average acceleration: {self.imu_stats['avg_acceleration']} m/s²")
        
        print("\n3. GPS Statistics:")
        print(f"   Total GPS fixes: {self.gps_stats['count']}")
        print(f"   Average position: {self.gps_stats['avg_latitude']:.6f}, {self.gps_stats['avg_longitude']:.6f}")
        print(f"   Average altitude: {self.gps_stats['avg_altitude']:.2f} m")
        
        # 重置统计数据
        self.pointcloud_stats['min_z'] = float('inf')
        self.pointcloud_stats['max_z'] = -float('inf')
        self.imu_stats['avg_angular_velocity'] = np.zeros(3)
        self.imu_stats['avg_acceleration'] = np.zeros(3)
        self.gps_stats['avg_latitude'] = 0
        self.gps_stats['avg_longitude'] = 0
        self.gps_stats['avg_altitude'] = 0
if __name__ == "__main__":
    rospy.init_node('lawnmower_data_monitor')
    monitor = DataMonitor()
    rospy.spin()