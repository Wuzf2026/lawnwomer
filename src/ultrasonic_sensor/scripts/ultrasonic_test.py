#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range

class UltrasonicTest:
    def __init__(self):
        rospy.init_node('ultrasonic_test', anonymous=True)
        self.distance_sub = rospy.Subscriber('/ultrasonic_distance', Float32, self.distance_callback)
        self.range_sub = rospy.Subscriber('/ultrasonic_range', Range, self.range_callback)
        rospy.loginfo("Ultrasonic sensor test started. Press Ctrl+C to exit.")
        rospy.spin()

    def distance_callback(self, msg):
        if msg.data != msg.data: # NaN check
            rospy.logwarn("Distance: Timeout")
        else:
            rospy.loginfo(f"Distance: {msg.data:.2f} m")

    def range_callback(self, msg):
        rospy.logdebug(f"Range: {msg.range:.2f} m (min: {msg.min_range}, max: {msg.max_range})")

if __name__ == '__main__':
    try:
        test = UltrasonicTest()
    except rospy.ROSInterruptException:
        pass