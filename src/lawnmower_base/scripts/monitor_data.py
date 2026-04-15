#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
from geometry_msgs.msg import Twist
import numpy as np

class LawnmowerDataMonitor:
    def __init__(self):
        rospy.init_node('lawnmower_data_monitor', anonymous=True)
        
        # 订阅统一发布的传感器数据和控制指令
        self.lidar_sub = rospy.Subscriber("/lawnmower/lidar", PointCloud2, self.lidar_callback)
        self.imu_sub = rospy.Subscriber("/lawnmower/imu", Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber("/lawnmower/gps", NavSatFix, self.gps_callback)
        self.cmd_vel_sub = rospy.Subscriber("/lawnmower/cmd_vel", Twist, self.cmd_vel_callback)

        # 数据缓存
        self.lidar_data = None
        self.imu_data = None
        self.gps_data = None
        self.cmd_vel_data = None

        # 监控频率
        self.rate = rospy.Rate(1.0)  # 1Hz

    def lidar_callback(self, msg):
        self.lidar_data = msg
        rospy.logdebug(f"Received LiDAR data: width={msg.width}, height={msg.height}")

    def imu_callback(self, msg):
        self.imu_data = msg
        rospy.logdebug(f"Received IMU data: angular_vel_x={msg.angular_velocity.x:.2f} rad/s")

    def gps_callback(self, msg):
        self.gps_data = msg
        rospy.logdebug(f"Received GPS data: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")

    def cmd_vel_callback(self, msg):
        self.cmd_vel_data = msg
        rospy.logdebug(f"Received control cmd: linear_x={msg.linear.x:.2f} m/s")

    def print_status(self):
        rospy.loginfo("\n===== Lawnmower Data Monitor =====")
        if self.lidar_data:
            rospy.loginfo(f"LiDAR: Frame={self.lidar_data.header.frame_id}, Stamp={self.lidar_data.header.stamp}")
        else:
            rospy.loginfo("LiDAR: No data received")
        
        if self.imu_data:
            rospy.loginfo(f"IMU: Angular Vel X={self.imu_data.angular_velocity.x:.2f} rad/s")
            rospy.loginfo(f"IMU: Linear Acc X={self.imu_data.linear_acceleration.x:.2f} m/s²")
        else:
            rospy.loginfo("IMU: No data received")
        
        if self.gps_data:
            rospy.loginfo(f"GPS: Lat={self.gps_data.latitude:.6f}, Lon={self.gps_data.longitude:.6f}")
            rospy.loginfo(f"GPS: Alt={self.gps_data.altitude:.2f} m")
        else:
            rospy.loginfo("GPS: No data received")
        
        if self.cmd_vel_data:
            rospy.loginfo(f"Control Cmd: Linear X={self.cmd_vel_data.linear.x:.2f} m/s")
            rospy.loginfo(f"Control Cmd: Angular Z={self.cmd_vel_data.angular.z:.2f} rad/s")
        else:
            rospy.loginfo("Control Cmd: No data received")
        rospy.loginfo("===================================\n")

    def run(self):
        while not rospy.is_shutdown():
            self.print_status()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        monitor = LawnmowerDataMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass