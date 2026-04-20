#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
class LiDARTool:
    def __init__(self):
        rospy.init_node('lidar_tool', anonymous=True)
        
        # 订阅激光雷达话题
        self.pcl_sub = rospy.Subscriber(
            '/hesai_jt128/pointcloud_raw', PointCloud2, self.pcl_callback
        )
        
        # 参数配置服务
        self.param_service = rospy.ServiceProxy(
            '/lidar/set_parameters', LiDARParameters
        )
        
        # 状态查询服务
        self.status_service = rospy.ServiceProxy(
            '/lidar/get_status', LiDARStatus
        )
        
        # 初始化可视化窗口
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
    
    def pcl_callback(self, msg):
        """点云数据回调函数"""
        # 从PointCloud2消息中提取点云数据
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        # 转换为numpy数组
        points_np = np.array(points)
        
        # 清除之前的绘图
        self.ax.clear()
        
        # 绘制点云俯视图
        self.ax.scatter(points_np[:, 0], points_np[:, 1], s=1, alpha=0.5)
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('LiDAR Point Cloud Top View')
        self.ax.grid(True, alpha=0.3)
        
        plt.pause(0.01)
    
    def set_parameters(self, params):
        """设置激光雷达参数"""
        try:
            response = self.param_service(params)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set LiDAR parameters: {e}")
            return False
    
    def get_status(self):
        """获取激光雷达状态"""
        try:
            response = self.status_service()
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get LiDAR status: {e}")
            return None
if __name__ == '__main__':
    try:
        tool = LiDARTool()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("LiDAR tool interrupted!")