#!/usr/bin/env python3
import rospy
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus
from datetime import datetime
class UM982Driver:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.gps_pub = rospy.Publisher('handsfree/rtk/fix', NavSatFix, queue_size=10)
        self.raw_pub = rospy.Publisher('handsfree/rtk/raw', std_msgs.msg.String, queue_size=10)
        
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo(f"Connected to UM982 on {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to connect to UM982: {e}")
            return False
        
    def read_data(self):
        if self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.raw_pub.publish(line)
                    if line.startswith('$GNGGA'):
                        self.parse_gga(line)
                    elif line.startswith('$GNRMC'):
                        self.parse_rmc(line)
            except Exception as e:
                rospy.logerr(f"Error reading from UM982: {e}")
                
    def parse_gga(self, gga_str):
        try:
            gga = pynmea2.parse(gga_str)
            fix_quality = gga.gps_qual
            status = NavSatStatus()
            
            # 映射GPS状态
            status_mapping = {
                0: NavSatStatus.STATUS_NO_FIX,
                1: NavSatStatus.STATUS_FIX,
                2: NavSatStatus.STATUS_SBAS_FIX,
                4: NavSatStatus.STATUS_GBAS_FIX,  # RTK Fixed
                5: NavSatStatus.STATUS_GBAS_FIX   # RTK Float
            }
            status.status = status_mapping.get(fix_quality, NavSatStatus.STATUS_NO_FIX)
            
            # 转换坐标
            lat = gga.latitude
            lon = gga.longitude
            
            # 创建NavSatFix消息
            fix_msg = NavSatFix()
            fix_msg.header.stamp = rospy.Time.now()
            fix_msg.header.frame_id = "gps"
            fix_msg.status = status
            fix_msg.latitude = lat
            fix_msg.longitude = lon
            fix_msg.altitude = gga.altitude
            fix_msg.position_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATE
            
            self.gps_pub.publish(fix_msg)
            
        except Exception as e:
            rospy.logwarn(f"Error parsing GGA: {e}")
            
    def parse_rmc(self, rmc_str):
        try:
            rmc = pynmea2.parse(rmc_str)
            if rmc.status == 'A':  # 有效数据
                # 可以添加更多解析逻辑
                pass
        except Exception as e:
            rospy.logwarn(f"Error parsing RMC: {e}")
            
if __name__ == '__main__':
    rospy.init_node('umd98x_driver_node')
    driver = UM982Driver()
    
    if driver.connect():
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            driver.read_data()
            rate.sleep()
    else:
        rospy.logerr("Failed to establish connection. Exiting...")