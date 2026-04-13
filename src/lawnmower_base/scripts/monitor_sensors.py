#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import curses
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix

class SensorMonitor:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        curses.curs_set(0)
        self.stdscr.nodelay(1)
        self.stdscr.timeout(100)
        
        # 初始化数据存储
        self.pc_data = None
        self.imu_data = None
        self.gps_data = None
        
        # 订阅传感器话题
        rospy.init_node('sensor_monitor', anonymous=True)
        rospy.Subscriber('/lawnmower/pointcloud', PointCloud2, self.pc_callback)
        rospy.Subscriber('/lawnmower/imu', Imu, self.imu_callback)
        rospy.Subscriber('/lawnmower/gps', NavSatFix, self.gps_callback)
        
        self.rate = rospy.Rate(10)

    def pc_callback(self, msg):
        self.pc_data = msg

    def imu_callback(self, msg):
        self.imu_data = msg

    def gps_callback(self, msg):
        self.gps_data = msg

    def display(self):
        while not rospy.is_shutdown():
            self.stdscr.clear()
            
            # 标题
            self.stdscr.addstr(0, 0, "=== Lawnmower Sensor Monitor ===", curses.A_BOLD)
            
            # Orbbec点云数据
            y = 2
            self.stdscr.addstr(y, 0, "Orbbec Gemini335:", curses.A_UNDERLINE)
            y += 1
            if self.pc_data:
                self.stdscr.addstr(y, 2, f"Resolution: {self.pc_data.width}x{self.pc_data.height}")
                y += 1
                self.stdscr.addstr(y, 2, f"Points: {len(self.pc_data.data)/self.pc_data.point_step:.0f}")
                y += 1
                self.stdscr.addstr(y, 2, f"Timestamp: {self.pc_data.header.stamp.to_sec():.2f}")
            else:
                self.stdscr.addstr(y, 2, "No data received", curses.A_REVERSE)
                y += 3
            
            # Hesai IMU数据
            y += 2
            self.stdscr.addstr(y, 0, "Hesai JT128 IMU:", curses.A_UNDERLINE)
            y += 1
            if self.imu_data:
                self.stdscr.addstr(y, 2, f"Angular Vel (rad/s): x={self.imu_data.angular_velocity.x:.3f}, y={self.imu_data.angular_velocity.y:.3f}, z={self.imu_data.angular_velocity.z:.3f}")
                y += 1
                self.stdscr.addstr(y, 2, f"Linear Acc (m/s²): x={self.imu_data.linear_acceleration.x:.3f}, y={self.imu_data.linear_acceleration.y:.3f}, z={self.imu_data.linear_acceleration.z:.3f}")
                y += 1
                self.stdscr.addstr(y, 2, f"Timestamp: {self.imu_data.header.stamp.to_sec():.2f}")
            else:
                self.stdscr.addstr(y, 2, "No data received", curses.A_REVERSE)
                y += 3
            
            # UM982 GPS数据
            y += 2
            self.stdscr.addstr(y, 0, "UM982 RTK GPS:", curses.A_UNDERLINE)
            y += 1
            if self.gps_data:
                self.stdscr.addstr(y, 2, f"Lat: {self.gps_data.latitude:.8f} °")
                y += 1
                self.stdscr.addstr(y, 2, f"Lon: {self.gps_data.longitude:.8f} °")
                y += 1
                self.stdscr.addstr(y, 2, f"Alt: {self.gps_data.altitude:.2f} m")
                y += 1
                self.stdscr.addstr(y, 2, f"Status: {self.gps_data.status.status}")
                y += 1
                self.stdscr.addstr(y, 2, f"Timestamp: {self.gps_data.header.stamp.to_sec():.2f}")
            else:
                self.stdscr.addstr(y, 2, "No data received", curses.A_REVERSE)
                y += 5
            
            # 帮助信息
            y += 2
            self.stdscr.addstr(y, 0, "Press 'q' to quit", curses.A_DIM)
            
            # 处理按键
            key = self.stdscr.getch()
            if key == ord('q'):
                break
            
            self.stdscr.refresh()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        stdscr = curses.initscr()
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        monitor = SensorMonitor(stdscr)
        monitor.display()
    finally:
        curses.endwin()