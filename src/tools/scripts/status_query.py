#!/usr/bin/env python3
import rospy
import yaml
from lawnmower_msgs.srv import LawnmowerStatus, LawnmowerStatusRequest

class StatusQuery:
    def __init__(self):
        rospy.init_node('status_query_node', anonymous=True)
        # 加载配置
        config_path = rospy.get_param("tools/config_path")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)['status_query']
        self.query_type = self.config['query_type']
        self.freq = self.config['query_frequency']
        
        # 等待服务启动
        rospy.wait_for_service('/lawnmower/status_query')
        self.status_service = rospy.ServiceProxy('/lawnmower/status_query', LawnmowerStatus)
        
        # 定时查询
        self.timer = rospy.Timer(rospy.Duration(1/self.freq), self.query_status)

    def query_status(self, event):
        """查询设备状态"""
        req = LawnmowerStatusRequest()
        req.query_type = self.query_type
        try:
            res = self.status_service(req)
            rospy.loginfo("=== Lawnmower Status ===")
            rospy.loginfo(f"Success: {res.success}")
            rospy.loginfo(f"Message: {res.status_msg}")
            if self.query_type in [0,3]:
                rospy.loginfo(f"Left wheel speed: {res.left_wheel_speed:.2f} m/s")
                rospy.loginfo(f"Right wheel speed: {res.right_wheel_speed:.2f} m/s")
            if self.query_type in [1,3]:
                rospy.loginfo(f"IMU angular z: {res.imu_angular_z:.2f} rad/s")
            if self.query_type in [2,3]:
                rospy.loginfo(f"GPS lat: {res.gps_lat:.6f}, lon: {res.gps_lon:.6f}")
            rospy.loginfo("========================\n")
        except rospy.ServiceException as e:
            rospy.logerr(f"Status query failed: {str(e)}")

if __name__ == '__main__':
    try:
        query = StatusQuery()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Status query node interrupted!")
    except Exception as e:
        rospy.logerr(f"Status query error: {str(e)}")