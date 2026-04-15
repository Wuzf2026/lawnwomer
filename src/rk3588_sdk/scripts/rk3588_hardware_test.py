#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Int32
from std_msgs.msg import String

class RK3588HardwareTest:
    def __init__(self):
        rospy.init_node('rk3588_hardware_test', anonymous=True)
        self.gpio_pub = rospy.Publisher('/set_gpio', Bool, queue_size=10)
        self.gpio_sub = rospy.Subscriber('/gpio_value', Int32, self.gpio_callback)
        self.uart_sub = rospy.Subscriber('/uart_data', String, self.uart_callback)
        self.rate = rospy.Rate(1) # 1Hz

    def gpio_callback(self, msg):
        rospy.loginfo(f"GPIO Value: {msg.data}")

    def uart_callback(self, msg):
        rospy.loginfo(f"UART Data: {msg.data}")

    def run(self):
        while not rospy.is_shutdown():
            # 切换GPIO状态
            self.gpio_pub.publish(Bool(data=True))
            rospy.loginfo("Set GPIO to HIGH")
            self.rate.sleep()
            self.gpio_pub.publish(Bool(data=False))
            rospy.loginfo("Set GPIO to LOW")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        test = RK3588HardwareTest()
        test.run()
    except rospy.ROSInterruptException:
        pass