#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import re
import time
import threading
import serial
import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus
# 定位状态映射
FIX_TYPE_MAPPING = {
    0: ("Invalid Fix", NavSatStatus.STATUS_NO_FIX, 10000.0),
    1: ("GPS Fix (SPS)", NavSatStatus.STATUS_FIX, 1.0),
    2: ("DGPS Fix", NavSatStatus.STATUS_SBAS_FIX, 0.5),
    3: ("PPS Fix", NavSatStatus.STATUS_NO_FIX, 10000.0),
    4: ("RTK Fixed", NavSatStatus.STATUS_GBAS_FIX, 0.01),
    5: ("RTK Float", NavSatStatus.STATUS_GBAS_FIX, 0.1),
    6: ("Estimated", NavSatStatus.STATUS_FIX, 5.0),
    7: ("Manual", NavSatStatus.STATUS_GBAS_FIX, 0.01),
    8: ("Simulation", NavSatStatus.STATUS_FIX, 10000.0)
}
GNRMC_MODE_MAPPING = {
    'A': 'Autonomous Mode', 'D': 'Differential Mode', 
    'E': 'INS Mode', 'F': 'RTK Float',
    'M': 'Manual Input Mode', 'N': 'No Fix',
    'P': 'Precision Mode', 'R': 'RTK Fixed',
    'S': 'Simulator Mode', 'V': 'Invalid Mode'
}
class RTKTagDriver(object):
    def __init__(self):
        rospy.init_node('tag_driver', anonymous=False)
        self.port = rospy.get_param("~port", "/dev/HFRobotRTK")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.timeout = rospy.get_param("~timeout", 0.1)
        
        # 发布者初始化
        self.raw_pub = rospy.Publisher("handsfree/rtk/raw", String, queue_size=10)
        self.gnss_pub = rospy.Publisher("handsfree/rtk/gnss", NavSatFix, queue_size=10)
        self.speed_pub = rospy.Publisher("handsfree/rtk/speed", Float64, queue_size=10)
        self.cog_pub = rospy.Publisher("handsfree/rtk/cog", Float64, queue_size=10)
        self.heading_pub = rospy.Publisher("handsfree/rtk/heading", Float64, queue_size=10)
        
        self._stop_event = threading.Event()
        self._read_thread = None
        self._ser = None
    def run(self):
        while not rospy.is_shutdown():
            try:
                self._ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
                rospy.loginfo("Serial opened: %s", self.port)
            except serial.SerialException as e:
                rospy.logerr("Failed to open serial: %s. Retry in 1s...", e)
                time.sleep(1.0)
                continue
            
            self._stop_event.clear()
            self._read_thread = threading.Thread(target=self._read_loop, name="nmea_reader")
            self._read_thread.daemon = True
            self._read_thread.start()
            
            while not rospy.is_shutdown() and self._read_thread.is_alive():
                time.sleep(0.1)
            self._cleanup_serial()
            if rospy.is_shutdown():
                break
            rospy.loginfo("Reconnecting in 1s...")
            time.sleep(1.0)
    def _read_loop(self):
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            try:
                line = self._ser.readline()
                if not line:
                    time.sleep(0.001)
                    continue
                
                line = line.decode('utf-8', 'ignore').strip()
                self.raw_pub.publish(line)
                
                parsed = self._parse_nmea_sentence(line)
                if not parsed:
                    continue
                
                # 处理GGA消息
                if parsed['type'] == 'GNGGA':
                    lat = parsed.get('latitude')
                    lon = parsed.get('longitude')
                    alt = parsed.get('altitude')
                    fix_q = parsed.get('fix_quality')
                    
                    status_str, status_value, cov_m = FIX_TYPE_MAPPING.get(
                        fix_q, ("Unknown", NavSatStatus.STATUS_NO_FIX, 10000.0))
                    
                    navsat = NavSatFix()
                    navsat.header.stamp = parsed['time']
                    navsat.header.frame_id = "gps"
                    navsat.status.status = status_value
                    navsat.status.service = NavSatStatus.SERVICE_GPS
                    navsat.latitude = lat
                    navsat.longitude = lon
                    navsat.altitude = alt if alt is not None else 0.0
                    
                    var = (cov_m ** 2)
                    navsat.position_covariance = [
                        var, 0.0, 0.0,
                        0.0, var, 0.0,
                        0.0, 0.0, var
                    ]
                    navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    self.gnss_pub.publish(navsat)
                
                # 处理RMC消息
                elif parsed['type'] == 'GNRMC':
                    speed_mps = parsed.get('speed_mps', 0.0)
                    rmc_status = parsed.get('status', 'V')
                    cog_deg = parsed.get('cog_deg', 0.0)
                    
                    if rmc_status != 'V':
                        self.speed_pub.publish(Float64(data=speed_mps))
                        self.cog_pub.publish(Float64(data=cog_deg))
                
                # 处理THS消息
                elif parsed['type'] == 'GNTHS':
                    heading = parsed.get('heading', None)
                    valid = parsed.get('valid', False)
                    if heading is not None and valid:
                        self.heading_pub.publish(Float64(data=heading))
            except Exception as e:
                rospy.logerr("Error in read loop: %s", e)
                self._stop_event.set()
    @staticmethod
    def _parse_nmea_sentence(sentence):
        # NMEA句子解析逻辑（支持GNGGA、GNRMC、GNTHS）
        try:
            if sentence.startswith('$GNGGA'):
                parts = sentence.split(',')
                if len(parts) < 10:
                    return None
                time_utc = parts[1]
                lat = RTKTagDriver._dms_to_decimal(parts[2], parts[3])
                lon = RTKTagDriver._dms_to_decimal(parts[4], parts[5])
                fix_q = int(parts[6] or 0)
                num_sats = int(parts[7] or 0)
                hdop = float(parts[8]) if parts[8] != '' else 99.9
                alt = float(parts[9]) if parts[9] != '' else 0.0
                ros_time = RTKTagDriver._nmea_time_to_ros(time_utc)
                
                return {
                    'type': 'GNGGA', 'time': ros_time,
                    'latitude': lat, 'longitude': lon,
                    'altitude': alt, 'fix_quality': fix_q,
                    'num_sats': num_sats, 'hdop': hdop
                }
            elif sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
                parts = sentence.split(',')
                if len(parts) < 12:
                    return None
                time_utc = parts[1]
                status = parts[2]
                lat = RTKTagDriver._dms_to_decimal(parts[3], parts[4])
                lon = RTKTagDriver._dms_to_decimal(parts[5], parts[6])
                speed_knots = float(parts[7]) if parts[7] != '' else 0.0
                cog_deg = float(parts[8]) if parts[8] != '' else 0.0
                ros_time = RTKTagDriver._nmea_time_to_ros(time_utc)
                speed_mps = speed_knots * 0.514444
                
                return {
                    'type': 'GNRMC', 'time': ros_time,
                    'status': status, 'latitude': lat,
                    'longitude': lon, 'speed_mps': speed_mps,
                    'cog_deg': cog_deg
                }
            elif sentence.startswith('$GNTHS') or sentence.startswith('GPTHS'):
                parts = sentence.split(',')
                if len(parts) < 3:
                    return None
                heading = float(parts[1]) if parts[1] != '' else None
                valid = parts[2].startswith('A') if len(parts[2]) > 0 else False
                
                return {'type': 'GNTHS', 'heading': heading, 'valid': valid}
            return None
        except Exception as e:
            rospy.logerr("NMEA parse error: %s", e)
            return None
    @staticmethod
    def _nmea_time_to_ros(nmea_time):
        from datetime import datetime
        if not nmea_time:
            return rospy.Time.now()
        try:
            utc_now = datetime.utcnow()
            hours = int(nmea_time[0:2])
            minutes = int(nmea_time[2:4])
            seconds = float(nmea_time[4:]) if len(nmea_time) > 4 else 0.0
            dt = datetime(utc_now.year, utc_now.month, utc_now.day, hours, minutes, int(seconds), int((seconds % 1) * 1e6))
            epoch = datetime(1970, 1, 1)
            return rospy.Time.from_sec((dt - epoch).total_seconds())
        except Exception as e:
            rospy.logerr("NMEA time parse error: %s", e)
            return rospy.Time.now()
    @staticmethod
    def _dms_to_decimal(dms, direction):
        try:
            if not dms:
                return None
            m = re.match(r"^(\d+)(\d\d\.\d+)$", dms)
            if not m:
                return None
            degrees = float(m.group(1))
            minutes = float(m.group(2))
            decimal = degrees + minutes / 60.0
            if direction in ('S', 'W'):
                decimal = -decimal
            return decimal
        except Exception:
            return None
if __name__ == '__main__':
    try:
        node = RTKTagDriver()
        node.run()
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        rospy.logerr("Fatal error: %s", e)
    finally:
        try:
            node.stop()
        except Exception:
            pass
        rospy.loginfo("Exit.")