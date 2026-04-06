#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
class DataRecorder:
    def __init__(self):
        rospy.init_node('data_recorder', anonymous=True)
        
        # 设置输出文件路径
        self.bag_file = rosbag.Bag('lawnmower_data.bag', 'w')
        
        # 订阅所有传感器话题
        rospy.Subscriber('/orbbec/camera/depth_points', PointCloud2, self.depth_callback)
        rospy.Subscriber('/orbbec/camera/imu', Imu, self.imu_callback)
        rospy.Subscriber('/hesai_lidar/points', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/handsfree_rtk/fix', NavSatFix, self.rtk_callback)
        
        rospy.loginfo("数据记录器已启动，按Ctrl+C停止")
    
    def depth_callback(self, msg):
        """深度相机数据回调"""
        self.bag_file.write('/orbbec/camera/depth_points', msg)
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        self.bag_file.write('/orbbec/camera/imu', msg)
    
    def lidar_callback(self, msg):
        """激光雷达数据回调"""
        self.bag_file.write('/hesai_lidar/points', msg)
    
    def rtk_callback(self, msg):
        """RTK定位数据回调"""
        self.bag_file.write('/handsfree_rtk/fix', msg)
    
    def __del__(self):
        """析构函数，关闭bag文件"""
        self.bag_file.close()
        rospy.loginfo("数据记录完成，文件已保存到lawnmower_data.bag")
if __name__ == '__main__':
    try:
        recorder = DataRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass