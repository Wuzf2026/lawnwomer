#!/usr/bin/env python
import rospy
import serial
import threading
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float64
from datetime import datetime
class RTKDriver:
    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/ttyUSB1')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.ser = None
        
        # 发布话题
        self.raw_pub = rospy.Publisher('handsfree/rtk/raw', String, queue_size=10)
        self.gnss_pub = rospy.Publisher('handsfree/rtk/gnss', NavSatFix, queue_size=10)
        self.speed_pub = rospy.Publisher('handsfree/rtk/speed', Float64, queue_size=10)
        self.cog_pub = rospy.Publisher('handsfree/rtk/cog', Float64, queue_size=10)
        self.heading_pub = rospy.Publisher('handsfree/rtk/heading', Float64, queue_size=10)
        
        # 启动串口线程
        self.running = True
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.start()
        
        # GNGGA状态映射
        self.gngga_status_map = {
            0: ("Invalid Fix", NavSatFix.STATUS_NO_FIX, 10000.0),
            1: ("GPS Fix (SPS)", NavSatFix.STATUS_FIX, 1.0),
            2: ("DGPS Fix", NavSatFix.STATUS_SBAS_FIX, 0.5),
            3: ("PPS Fix", NavSatFix.STATUS_NO_FIX, 10000.0),
            4: ("RTK Fixed", NavSatFix.STATUS_GBAS_FIX, 0.01),
            5: ("RTK Float", NavSatFix.STATUS_GBAS_FIX, 0.1),
            6: ("Estimated", NavSatFix.STATUS_FIX, 5.0),
            7: ("Manual", NavSatFix.STATUS_GBAS_FIX, 0.01),
            8: ("Simulation", NavSatFix.STATUS_FIX, 10000.0)
        }
        
        # GNRMC状态映射
        self.gnrmc_status_map = {
            'A': 'Autonomous Mode',
            'D': 'Differential Mode', 
            'E': 'INS Mode',
            'F': 'RTK Float',
            'M': 'Manual Input Mode',
            'N': 'No Fix',
            'P': 'Precision Mode',
            'R': 'RTK Fixed',
            'S': 'Simulator Mode',
            'V': 'Invalid Mode'
        }
    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo(f"RTK serial port opened: {self.port}")
        except Exception as e:
            rospy.logerr(f"Failed to open RTK serial port: {e}")
    def read_serial(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                self.open_serial()
                rospy.sleep(1)
                continue
            
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.parse_nmea(line)
            except Exception as e:
                rospy.logerr(f"Serial read error: {e}")
                self.ser.close()
                rospy.sleep(1)
    def parse_nmea(self, line):
        self.raw_pub.publish(line)
        
        if line.startswith('$GNGGA'):
            self.parse_gga(line)
        elif line.startswith('$GNRMC'):
            self.parse_rmc(line)
    def parse_gga(self, gga_str):
        parts = gga_str.split(',')
        if len(parts) < 15:
            return
        
        try:
            status = int(parts[6])
            if status in self.gngga_status_map:
                status_name, ros_status, hdop = self.gngga_status_map[status]
                
                # 解析经纬度
                lat = self.parse_lat(parts[2])
                lon = self.parse_lon(parts[4])
                alt = float(parts[9]) if parts[9] else 0.0
                
                # 创建NavSatFix消息
                msg = NavSatFix()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'gps'
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = alt
                msg.status.status = ros_status
                msg.status.service = NavSatFix.SERVICE_GPS
                msg.position_covariance[0] = hdop * hdop  # 简化的协方差估计
                
                self.gnss_pub.publish(msg)
        except Exception as e:
            rospy.logwarn(f"GGA parse error: {e}")
    def parse_rmc(self, rmc_str):
        parts = rmc_str.split(',')
        if len(parts) < 13:
            return
        
        try:
            if parts[2] == 'A':  # 有效数据
                # 速度（节转米/秒）
                speed_knots = float(parts[7]) if parts[7] else 0.0
                speed_ms = speed_knots * 0.514444
                self.speed_pub.publish(speed_ms)
                
                # 航向（COG）
                cog = float(parts[8]) if parts[8] else 0.0
                self.cog_pub.publish(cog)
                
                # 航向角（Heading）
                heading = float(parts[9]) if parts[9] else 0.0
                self.heading_pub.publish(heading)
        except Exception as e:
            rospy.logwarn(f"RMC parse error: {e}")
    def parse_lat(self, lat_str):
        """解析纬度字符串"""
        degrees = float(lat_str[:2])
        minutes = float(lat_str[2:])
        return degrees + minutes / 60.0
    def parse_lon(self, lon_str):
        """解析经度字符串"""
        degrees = float(lon_str[:3])
        minutes = float(lon_str[3:])
        return degrees + minutes / 60.0
    def shutdown(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.serial_thread.join()
if __name__ == '__main__':
    rospy.init_node('handsfree_rtk_node', anonymous=True)
    driver = RTKDriver()
    rospy.on_shutdown(driver.shutdown)
    rospy.spin()