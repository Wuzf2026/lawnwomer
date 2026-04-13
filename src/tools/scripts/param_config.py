#!/usr/bin/env python3
import rospy
import yaml

def load_config(config_path):
    """加载配置文件"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def set_ros_params(config):
    """设置ROS参数"""
    # 设置电机参数
    rospy.set_param("/motor_control/max_linear_speed", config['param_config']['motor']['max_linear_speed'])
    rospy.set_param("/motor_control/max_angular_speed", config['param_config']['motor']['max_angular_speed'])
    # 设置串口参数
    rospy.set_param("/motor_control/serial/baudrate", config['param_config']['serial']['baudrate'])
    rospy.set_param("/motor_control/serial/port", config['param_config']['serial']['port'])
    rospy.loginfo("Params set successfully!")

def update_param(param_name, value):
    """动态更新参数"""
    rospy.set_param(param_name, value)
    rospy.loginfo(f"Updated param {param_name} to {value}")

if __name__ == '__main__':
    rospy.init_node('param_config_node', anonymous=True)
    try:
        # 加载配置文件
        config_path = rospy.get_param("tools/config_path")
        config = load_config(config_path)
        # 初始化参数
        set_ros_params(config)
        
        # 示例：动态修改参数（可通过命令行/终端输入扩展）
        rospy.sleep(2)
        update_param("/motor_control/max_linear_speed", 0.6)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Param config node interrupted!")
    except Exception as e:
        rospy.logerr(f"Param config error: {str(e)}")