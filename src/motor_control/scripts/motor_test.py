#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class MotorTest:
    def __init__(self):
        rospy.init_node('motor_test', anonymous=True)
        self.speed_pub = rospy.Publisher('/set_speed', Float32, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.speed_sub = rospy.Subscriber('/motor_speed', Float32, self.speed_callback)
        self.rate = rospy.Rate(1) # 1Hz

    def speed_callback(self, msg):
        rospy.loginfo(f"Current Motor Speed: {msg.data:.2f} RPM")

    def run(self):
        rospy.loginfo("Starting motor test...")
        # 测试1：设置固定转速
        for speed in [0, 50, 100, 50, 0, -50, -100, -50, 0]:
            if rospy.is_shutdown():
                break
            self.speed_pub.publish(Float32(data=speed))
            rospy.loginfo(f"Set motor speed to {speed} RPM")
            rospy.sleep(2)
        
        # 测试2：通过cmd_vel控制
        rospy.loginfo("Testing cmd_vel control...")
        twist = Twist()
        twist.linear.x = 0.5 # 0.5m/s = 50RPM
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(3)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Motor test completed")

if __name__ == '__main__':
    try:
        test = MotorTest()
        test.run()
    except rospy.ROSInterruptException:
        pass