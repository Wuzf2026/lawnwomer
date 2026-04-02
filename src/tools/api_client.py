#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import requests
from srv.srv import *

class APIClient:
    def __init__(self):
        rospy.init_node('api_client')
        self.api_port = rospy.get_param('/api_server/server_port', 8088)
        self.http_url = f"http://localhost:{self.api_port}"
        rospy.loginfo(f"✅ API客户端启动，HTTP端口: {self.api_port}")

    def test_all_ros_services(self):
        rospy.loginfo("===== 测试ROS服务 =====")
        for sensor in ['orbbec', 'hesai', 'um982']:
            try:
                cli = rospy.ServiceProxy('/lawnmower/api/get_sensor_status', GetSensorStatus)
                res = cli(sensor)
                rospy.loginfo(f"[{sensor}] 状态: {res.is_connected}")
            except:
                rospy.logerr(f"[{sensor}] 服务调用失败")

    def test_http_api(self):
        try:
            res = requests.get(f"{self.http_url}/health", timeout=3)
            rospy.loginfo(f"HTTP API健康检查: {res.json()}")
        except:
            rospy.logwarn("HTTP API未启用")

if __name__ == "__main__":
    client = APIClient()
    client.test_all_ros_services()
    client.test_http_api()