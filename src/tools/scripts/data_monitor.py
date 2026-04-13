#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, NavSatFix

class DataMonitor:
    def __init__(self):
        rospy.init_node('data_monitor_node', anonymous=True)
        # 加载配置
        self.config = rospy.get_param("tools/config_path")
        with open(self.config, 'r') as f:
            self.config = yaml.safe_load(f)['data_monitor']
        self.topics = self.config['topics']
        self.freq = self.config['update_frequency']
        self.data_cache = {}
        
        # 订阅所有监控话题
        for topic in self.topics:
            # 根据话题类型自动匹配消息类型
            if "imu" in topic:
                rospy.Subscriber(topic, Imu, self.callback, callback_args=topic)
            elif "gps" in topic:
                rospy.Subscriber(topic, NavSatFix, self.callback, callback_args=topic)
            elif "speed" in topic:
                rospy.Subscriber(topic, Float64, self.callback, callback_args=topic)
            else:
                rospy.Subscriber(topic, rospy.AnyMsg, self.callback, callback_args=topic)
        
        # 定时打印监控数据
        self.timer = rospy.Timer(rospy.Duration(1/self.freq), self.print_data)

    def callback(self, msg, topic):
        """话题回调：缓存数据"""
        self.data_cache[topic] = msg

    def print_data(self, event):
        """打印监控数据"""
        rospy.loginfo("=== Data Monitor (RK3588) ===")
        for topic, msg in self.data_cache.items():
            if "imu" in topic:
                rospy.loginfo(f"{topic}: angular_z={msg.angular_velocity.z:.2f} rad/s")
            elif "gps" in topic:
                rospy.loginfo(f"{topic}: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")
            elif "speed" in topic:
                rospy.loginfo(f"{topic}: {msg.data:.2f} m/s")
            else:
                rospy.loginfo(f"{topic}: {msg}")
        rospy.loginfo("=============================\n")

if __name__ == '__main__':
    import yaml
    try:
        monitor = DataMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Data monitor node interrupted!")
    except Exception as e:
        rospy.logerr(f"Data monitor error: {str(e)}")