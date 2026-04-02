#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import yaml
import os
from pathlib import Path

class ParamConfigurator:
    def __init__(self):
        rospy.init_node('param_configurator')
        self.config_root = Path(rospy.get_pkg_dir('config')) / 'config'
        self.config_files = {
            'orbbec': 'orbbec_config.yaml',
            'hesai': 'hesai_config.yaml',
            'um982': 'um982_config.yaml',
            'api': 'api_config.yaml'
        }
        rospy.loginfo("✅ 参数配置工具初始化完成")

    def load_config(self, sensor_type):
        path = self.config_root / self.config_files.get(sensor_type)
        if not path.exists():
            rospy.logerr(f"配置文件不存在: {path}")
            return None
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    def save_config(self, sensor_type, data):
        path = self.config_root / self.config_files.get(sensor_type)
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True)
        rospy.loginfo(f"配置已保存: {path}")

    def modify_param(self, sensor_type, param_key, param_value):
        config = self.load_config(sensor_type)
        if not config: return False
        
        if sensor_type == 'orbbec':
            root_key = 'orbbec_gemini335'
        elif sensor_type == 'hesai':
            root_key = 'hesai_jt128'
        elif sensor_type == 'um982':
            root_key = 'um982_rtk'
        else:
            root_key = 'api_server'
        
        key_list = param_key.split('.')
        node = config[root_key]
        
        for k in key_list[:-1]:
            if k not in node:
                rospy.logerr(f"参数不存在: {param_key}")
                return False
            node = node[k]
        node[key_list[-1]] = type(node[key_list[-1]])(param_value)
        
        self.save_config(sensor_type, config)
        return True

    def print_params(self, sensor_type):
        config = self.load_config(sensor_type)
        if config:
            rospy.loginfo(f"\n{sensor_type} 配置参数:\n{yaml.dump(config, indent=2)}")

if __name__ == "__main__":
    tool = ParamConfigurator()
    tool.print_params('um982')
    rospy.spin()