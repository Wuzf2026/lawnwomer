#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2,Imu
from nav_msgs.msg import NavSatFix
class Monitor:
    def __init__(self):
        rospy.init_node("sensor_data_monitor")
        rospy.Subscriber("/sensor_data/orbbec/pointcloud2",PointCloud2,self.cloud_cb)
        rospy.Subscriber("/sensor_data/hesai/imu",Imu,self.imu_cb)
        rospy.Subscriber("/sensor_data/um982/gps",NavSatFix,self.gps_cb)
    def cloud_cb(self,msg):
        rospy.loginfo(f"[Orbbec] 点云分辨率:{msg.width}*{msg.height}")
    def imu_cb(self,msg):
        rospy.loginfo(f"[Hesai] 角速度Z:{msg.angular_velocity.z:.2f}")
    def gps_cb(self,msg):
        rospy.loginfo(f"[UM982] 纬度:{msg.latitude:.6f} 经度:{msg.longitude:.6f}")
    def run(self):
        rospy.spin()
if __name__=="__main__":
    m=Monitor()
    m.run()