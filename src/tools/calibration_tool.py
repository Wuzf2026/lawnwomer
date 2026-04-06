#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
class CameraCalibrationTool:
    def __init__(self):
        rospy.init_node('camera_calibration_tool', anonymous=True)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/orbbec/camera/rgb_image', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/orbbec/camera/camera_info', CameraInfo, self.camera_info_callback)
        
        # 标定参数
        self.board_size = (9, 6)  # 棋盘格内角点数量
        self.square_size = 0.025  # 棋盘格方块大小（米）
        
        # 存储图像和角点
        self.images = []
        self.obj_points = []  # 世界坐标系下的点
        self.img_points = []  # 图像坐标系下的点
        
        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
    
    def image_callback(self, msg):
        """图像数据回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)
            
            if ret:
                # 添加世界坐标系点
                objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2) * self.square_size
                
                self.obj_points.append(objp)
                self.img_points.append(corners)
                
                # 在图像上绘制角点
                cv2.drawChessboardCorners(cv_image, self.board_size, corners, ret)
                cv2.imshow('Calibration Image', cv_image)
                cv2.waitKey(500)
                
                rospy.loginfo(f"找到第{len(self.images)}张标定图像")
                self.images.append(cv_image)
        
        except Exception as e:
            rospy.logerr(f"图像回调错误: {e}")
    
    def camera_info_callback(self, msg):
        """相机信息回调"""
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
    
    def start_calibration(self):
        """开始标定"""
        if len(self.images) < 10:
            rospy.logwarn("需要至少10张标定图像")
            return
        
        # 执行标定
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, self.images[0].shape[::-1], None, None)
        
        if ret:
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
            self.rvecs = rvecs
            self.tvecs = tvecs
            
            # 保存标定结果
            calibration_data = {
                'camera_matrix': self.camera_matrix.tolist(),
                'dist_coeffs': self.dist_coeffs.tolist(),
                'rvecs': [rvec.tolist() for rvec in self.rvecs],
                'tvecs': [tvec.tolist() for tvec in self.tvecs]
            }
            
            import json
            with open('camera_calibration.json', 'w') as f:
                json.dump(calibration_data, f)
            
            rospy.loginfo("标定完成，结果已保存到camera_calibration.json")
        else:
            rospy.logerr("标定失败")
    
    def run(self):
        """运行标定工具"""
        while not rospy.is_shutdown():
            command = input("\n请输入命令（c:开始标定, q:退出）: ")
            
            if command == 'c':
                self.start_calibration()
            elif command == 'q':
                break
if __name__ == '__main__':
    try:
        calibration_tool = CameraCalibrationTool()
        calibration_tool.run()
    except rospy.ROSInterruptException:
        pass