#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
class SensorMonitor:
    def __init__(self):
        rospy.init_node('sensor_monitor', anonymous=True)
        
        # 订阅传感器话题
        rospy.Subscriber('/orbbec/camera/depth_points', PointCloud2, self.depth_callback)
        rospy.Subscriber('/orbbec/camera/imu', Imu, self.imu_callback)
        rospy.Subscriber('/hesai_lidar/points', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/handsfree_rtk/fix', NavSatFix, self.rtk_callback)
        
        # 初始化状态
        self.sensor_status = {
            'orbbec_depth': {'status': 'disconnected', 'points': 0},
            'orbbec_imu': {'status': 'disconnected'},
            'hesai_lidar': {'status': 'disconnected', 'points': 0},
            'rtk_fix': {'status': 'disconnected'}
        }
        
        # 启动监控循环
        self.monitor_loop()
    
    def depth_callback(self, msg):
        """深度相机数据回调"""
        self.sensor_status['orbbec_depth']['status'] = 'connected'
        self.sensor_status['orbbec_depth']['points'] = msg.width
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        self.sensor_status['orbbec_imu']['status'] = 'connected'
    
    def lidar_callback(self, msg):
        """激光雷达数据回调"""
        self.sensor_status['hesai_lidar']['status'] = 'connected'
        self.sensor_status['hesai_lidar']['points'] = msg.width
    
    def rtk_callback(self, msg):
        """RTK定位数据回调"""
        self.sensor_status['rtk_fix']['status'] = 'connected'
    
    def monitor_loop(self):
        """监控循环"""
        rate = rospy.Rate(1)  # 每秒更新一次
        
        while not rospy.is_shutdown():
            print("\n=== 传感器监控状态 ===")
            for sensor, status in self.sensor_status.items():
                print(f"{sensor}: {status['status']}")
                if 'points' in status:
                    print(f"  点云数量: {status['points']}")
            
            rate.sleep()
if __name__ == '__main__':
    try:
        SensorMonitor()
    except rospy.ROSInterruptException:
        pass