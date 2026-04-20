#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class CameraTool:
    def __init__(self):
        rospy.init_node('camera_tool', anonymous=True)
        self.bridge = CvBridge()
        
        # 订阅相机话题
        self.color_sub = rospy.Subscriber(
            '/gemini335/color/image_raw', Image, self.color_callback
        )
        self.depth_sub = rospy.Subscriber(
            '/gemini335/depth/image_raw', Image, self.depth_callback
        )
        
        # 参数配置服务
        self.param_service = rospy.ServiceProxy(
            '/camera/set_parameters', CameraParameters
        )
        
        # 状态查询服务
        self.status_service = rospy.ServiceProxy(
            '/camera/get_status', CameraStatus
        )
    
    def color_callback(self, msg):
        """彩色图像回调函数"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('Color Image', cv_image)
        cv2.waitKey(1)
    
    def depth_callback(self, msg):
        """深度图像回调函数"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        cv2.imshow('Depth Image', cv_image)
        cv2.waitKey(1)
    
    def set_parameters(self, params):
        """设置相机参数"""
        try:
            response = self.param_service(params)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set camera parameters: {e}")
            return False
    
    def get_status(self):
        """获取相机状态"""
        try:
            response = self.status_service()
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get camera status: {e}")
            return None
if __name__ == '__main__':
    try:
        tool = CameraTool()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Camera tool interrupted!")