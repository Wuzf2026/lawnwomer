#!/usr/bin/env python3
import rospy
import yaml

def set_lawnmower_params():
    rospy.init_node('lawnmower_config_node', anonymous=True)
    
    # 加载默认配置
    default_config = {
        "lidar_topic": "/hesai_points",
        "imu_topic": "/imu/data",
        "gps_topic": "/gps/fix",
        "cmd_vel_topic": "/cmd_vel",
        "publish_freq": 10.0
    }

    # 从命令行参数加载配置文件（可选）
    if rospy.has_param("~config_file"):
        config_file = rospy.get_param("~config_file")
        try:
            with open(config_file, 'r') as f:
                custom_config = yaml.safe_load(f)
                default_config.update(custom_config)
            rospy.loginfo(f"Loaded custom config from {config_file}")
        except Exception as e:
            rospy.logerr(f"Failed to load config file: {e}, use default config")

    # 设置参数到ROS参数服务器
    for key, value in default_config.items():
        rospy.set_param(f"/lawnmower_base_node/{key}", value)
        rospy.loginfo(f"Set param {key} = {value}")

    rospy.loginfo("All parameters configured successfully!")

if __name__ == '__main__':
    try:
        set_lawnmower_params()
    except rospy.ROSInterruptException:
        pass