#!/usr/bin/env python3
import rospy
import argparse
def main():
    rospy.init_node("sensor_param_config")
    parser = argparse.ArgumentParser()
    parser.add_argument("--w",type=int,default=1280)
    parser.add_argument("--h",type=int,default=720)
    parser.add_argument("--imu",type=float,default=100.0)
    parser.add_argument("--tty",type=str,default="/dev/ttyUSB0")
    args = parser.parse_args()
    rospy.set_param("/sensor_data_collect/cloud_width",args.w)
    rospy.set_param("/sensor_data_collect/cloud_height",args.h)
    rospy.set_param("/sensor_data_collect/imu_rate",args.imu)
    rospy.set_param("/sensor_data_collect/gps_tty",args.tty)
    print("参数写入完成")
if __name__=="__main__":
    main()