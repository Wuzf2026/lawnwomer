#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import argparse
from msg.msg import RTKData, LidarData, StereoImage
from srv.srv import SetSensorParam, GetSensorStatus, CalibrateSensor

class SensorDebugger:
    def __init__(self):
        rospy.init_node('sensor_debugger', anonymous=True)
        
        self.rtk_sub = rospy.Subscriber("/lawnmower/um982/rtk_data", RTKData, self.rtk_callback)
        self.lidar_sub = rospy.Subscriber("/lawnmower/hesai/lidar_data", LidarData, self.lidar_callback)
        self.stereo_sub = rospy.Subscriber("/lawnmower/orbbec/stereo_image", StereoImage, self.stereo_callback)
        
        rospy.wait_for_service('/lawnmower/api/set_sensor_param')
        rospy.wait_for_service('/lawnmower/api/get_sensor_status')
        rospy.wait_for_service('/lawnmower/api/calibrate_sensor')
        
        self.set_param_cli = rospy.ServiceProxy('/lawnmower/api/set_sensor_param', SetSensorParam)
        self.get_status_cli = rospy.ServiceProxy('/lawnmower/api/get_sensor_status', GetSensorStatus)
        self.calib_cli = rospy.ServiceProxy('/lawnmower/api/calibrate_sensor', CalibrateSensor)
        
        rospy.loginfo("✅ 传感器调试工具初始化完成")
        self.print_help()

    def rtk_callback(self, msg):
        fix_state = ["未定位", "单点定位", "RTK固定解"][msg.fix_status] if msg.fix_status <3 else "未知"
        rospy.logdebug(f"[RTK] {fix_state} | 经纬度: {msg.latitude:.6f}, {msg.longitude:.6f} | 海拔: {msg.altitude:.2f}m")

    def lidar_callback(self, msg):
        rospy.logdebug(f"[雷达] 点云数量: {msg.point_num} | 测距范围: {min(msg.x):.2f}~{max(msg.x):.2f}m")

    def stereo_callback(self, msg):
        rospy.logdebug(f"[双目] 分辨率: {msg.left_image.width}x{msg.left_image.height} | 基线: {msg.baseline:.3f}m")

    def get_sensor_status(self, sensor_type):
        try:
            res = self.get_status_cli(sensor_type)
            rospy.loginfo(f"[{sensor_type}] 连接: {res.is_connected} | 状态: {res.status_detail} | 最后更新: {res.last_update}")
        except Exception as e:
            rospy.logerr(f"获取状态失败: {str(e)}")

    def set_sensor_param(self, sensor_type, key, value):
        try:
            res = self.set_param_cli(sensor_type, key, value)
            rospy.loginfo(f"设置参数: {res.success} | {res.message}")
        except Exception as e:
            rospy.logerr(f"设置参数失败: {str(e)}")

    def calibrate_sensor(self, sensor_type, calib_type):
        try:
            res = self.calib_cli(sensor_type, calib_type)
            rospy.loginfo(f"校准结果: {res.success} | 误差: {res.calib_error} | {res.message}")
        except Exception as e:
            rospy.logerr(f"校准失败: {str(e)}")

    def print_help(self):
        print("\n===== 调试指令说明 =====")
        print("1. status <传感器>  → 查询状态 (orbbec/hesai/um982)")
        print("2. set <传感器> <参数名> <参数值> → 设置参数")
        print("3. calib <传感器> <校准类型> → 校准传感器")
        print("4. quit → 退出工具\n")

    def run(self):
        while not rospy.is_shutdown():
            cmd = input("请输入指令: ").strip().split()
            if not cmd: continue
            if cmd[0] == "quit": break
            elif cmd[0] == "status" and len(cmd)==2:
                self.get_sensor_status(cmd[1])
            elif cmd[0] == "set" and len(cmd)==4:
                self.set_sensor_param(cmd[1], cmd[2], cmd[3])
            elif cmd[0] == "calib" and len(cmd)==3:
                self.calibrate_sensor(cmd[1], cmd[2])
            else:
                self.print_help()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true', help='开启调试日志')
    args = parser.parse_args()
    
    if args.debug:
        rospy.init_node('sensor_debugger', log_level=rospy.DEBUG)
    
    debugger = SensorDebugger()
    try:
        debugger.run()
    except rospy.ROSInterruptException:
        pass