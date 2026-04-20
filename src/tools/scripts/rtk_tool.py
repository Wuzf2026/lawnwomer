#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
class RTKTool:
    def __init__(self):
        rospy.init_node('rtk_tool', anonymous=True)
        
        # 订阅RTK话题
        self.gps_sub = rospy.Subscriber(
            '/um982/nmea_raw', String, self.nmea_callback
        )
        self.fix_sub = rospy.Subscriber(
            '/um982/fix', NavSatFix, self.fix_callback
        )
        
        # 参数配置服务
        self.param_service = rospy.ServiceProxy(
            '/rtk/set_parameters', RTKParameters
        )
        
        # 状态查询服务
        self.status_service = rospy.ServiceProxy(
            '/rtk/get_status', RTKStatus
        )
    
    def nmea_callback(self, msg):
        """NMEA原始数据回调函数"""
        rospy.loginfo(f"Received NMEA: {msg.data}")
    
    def fix_callback(self, msg):
        """GPS定位数据回调函数"""
        rospy.loginfo(f"GPS Fix: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}")
    
    def set_parameters(self, params):
        """设置RTK参数"""
        try:
            response = self.param_service(params)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set RTK parameters: {e}")
            return False
    
    def get_status(self):
        """获取RTK状态"""
        try:
            response = self.status_service()
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get RTK status: {e}")
            return None
if __name__ == '__main__':
    try:
        tool = RTKTool()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("RTK tool interrupted!")