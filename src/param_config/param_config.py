#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.client import Client
from lawnmower_base.cfg import LawnmowerBaseConfig
class ParameterConfigTool:
    def __init__(self):
        self.client = Client('/lawnmower_base', timeout=30)
        self.current_config = None
        
        # 注册动态参数回调
        self.client.update_configuration_callback = self.config_callback
    
    def config_callback(self, config):
        """动态参数更新回调函数"""
        self.current_config = config
        print("Parameter updated:")
        print(f"  pointcloud_publish_rate: {config.pointcloud_publish_rate} Hz")
        print(f"  imu_publish_rate: {config.imu_publish_rate} Hz")
        print(f"  gps_publish_rate: {config.gps_publish_rate} Hz")
        print(f"  enable_ekf: {config.enable_ekf}")
        print(f"  enable_tf_broadcast: {config.enable_tf_broadcast}")
    
    def get_current_config(self):
        """获取当前参数配置"""
        return self.client.get_configuration()
    
    def set_configuration(self, config_dict):
        """设置参数配置"""
        self.client.update_configuration(config_dict)
    
    def print_menu(self):
        """打印菜单"""
        print("\n=== Lawnmower Base Parameter Configuration ===")
        print("1. Show current configuration")
        print("2. Set pointcloud publish rate")
        print("3. Set IMU publish rate")
        print("4. Set GPS publish rate")
        print("5. Toggle EKF enable")
        print("6. Toggle TF broadcast")
        print("7. Save configuration to file")
        print("8. Load configuration from file")
        print("9. Exit")
    
    def run(self):
        """运行配置工具"""
        while True:
            self.print_menu()
            choice = input("Enter your choice: ")
            
            if choice == '1':
                config = self.get_current_config()
                print("\nCurrent configuration:")
                for key, value in config.items():
                    print(f"  {key}: {value}")
            
            elif choice == '2':
                rate = float(input("Enter new pointcloud publish rate (Hz): "))
                self.set_configuration({'pointcloud_publish_rate': rate})
            
            elif choice == '3':
                rate = float(input("Enter new IMU publish rate (Hz): "))
                self.set_configuration({'imu_publish_rate': rate})
            
            elif choice == '4':
                rate = float(input("Enter new GPS publish rate (Hz): "))
                self.set_configuration({'gps_publish_rate': rate})
            
            elif choice == '5':
                enable = not self.current_config.enable_ekf
                self.set_configuration({'enable_ekf': enable})
                print(f"EKF {'enabled' if enable else 'disabled'}")
            
            elif choice == '6':
                enable = not self.current_config.enable_tf_broadcast
                self.set_configuration({'enable_tf_broadcast': enable})
                print(f"TF broadcast {'enabled' if enable else 'disabled'}")
            
            elif choice == '7':
                import json
                filename = input("Enter filename to save: ")
                with open(filename, 'w') as f:
                    json.dump(self.current_config, f, indent=2)
                print(f"Configuration saved to {filename}")
            
            elif choice == '8':
                import json
                filename = input("Enter filename to load: ")
                with open(filename, 'r') as f:
                    config = json.load(f)
                self.set_configuration(config)
                print(f"Configuration loaded from {filename}")
            
            elif choice == '9':
                print("Exiting parameter configuration tool...")
                break
            
            else:
                print("Invalid choice. Please try again.")
if __name__ == "__main__":
    rospy.init_node('lawnmower_param_config_tool')
    tool = ParameterConfigTool()
    tool.run()