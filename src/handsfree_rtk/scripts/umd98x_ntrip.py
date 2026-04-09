#!/usr/bin/env python3
import rospy
import requests
import serial
from threading import Thread
import time
class NTRIPClient:
    def __init__(self, server='120.253.239.161', port=8002, 
                 username='ctea952', password='cm286070', 
                 mountpoint='RTCM33_GRCE', serial_port='/dev/ttyUSB1'):
        self.server = server
        self.port = port
        self.username = username
        self.password = password
        self.mountpoint = mountpoint
        self.serial_port = serial_port
        self.ser = None
        self.running = False
        
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
            return True
        except Exception as e:
            rospy.logerr(f"Serial connection failed: {e}")
            return False
        
    def start_ntrip(self):
        if not self.ser:
            rospy.logerr("Serial port not connected.")
            return
            
        # NTRIP请求头
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP um982_ntrip',
            'Connection': 'close',
            'Authorization': 'Basic ' + base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
        }
        
        # 构建请求URL
        url = f"http://{self.server}:{self.port}/{self.mountpoint}"
        
        try:
            response = requests.get(url, headers=headers, stream=True, timeout=5)
            if response.status_code == 200:
                rospy.loginfo("NTRIP connection established.")
                self.running = True
                
                # 启动数据接收线程
                data_thread = Thread(target=self.receive_data, args=(response,))
                data_thread.daemon = True
                data_thread.start()
                
                # 定期发送GGA请求
                gga_thread = Thread(target=self.send_gga_request)
                gga_thread.daemon = True
                gga_thread.start()
            else:
                rospy.logerr(f"NTRIP connection failed with status code {response.status_code}")
                
        except Exception as e:
            rospy.logerr(f"NTRIP connection error: {e}")
            
    def receive_data(self, response):
        try:
            for chunk in response.iter_content(chunk_size=1024):
                if self.running:
                    self.ser.write(chunk)
                else:
                    break
        except Exception as e:
            rospy.logerr(f"Error receiving NTRIP data: {e}")
            self.stop()
            
    def send_gga_request(self):
        while self.running:
            try:
                self.ser.write(b'$GPGGA,1\r\n')  # 每10秒发送一次GGA请求
            except Exception as e:
                rospy.logwarn(f"Error sending GGA request: {e}")
            time.sleep(10)
            
    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        rospy.loginfo("NTRIP client stopped.")
        
if __name__ == '__main__':
    rospy.init_node('umd98x_ntrip_node')
    
    # 从参数服务器获取配置
    server = rospy.get_param('~ntrip_server', '120.253.239.161')
    port = rospy.get_param('~ntrip_port', 8002)
    username = rospy.get_param('~ntrip_username', 'ctea952')
    password = rospy.get_param('~ntrip_password', 'cm286070')
    mountpoint = rospy.get_param('~ntrip_mountpoint', 'RTCM33_GRCE')
    
    client = NTRIPClient(server, port, username, password, mountpoint)
    
    if client.connect_serial():
        client.start_ntrip()
        rospy.spin()
        client.stop()
    else:
        rospy.logerr("Exiting due to serial connection failure.")