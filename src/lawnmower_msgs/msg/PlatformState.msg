float32 battery_percentage  # 电池电量百分比
float32 motor_current[2]    # 左右电机电流
float32 motor_temperature[2] # 左右电机温度
float32 speed               # 当前速度 (m/s)
float32 heading             # 当前航向 (rad)
string state                # 平台状态 (Idle, Moving, Cutting, Error)
# SystemStatus.srv
uint8 query_type  # 0:电机状态, 1:传感器状态, 2:系统状态, 3:全部状态
---
bool success
string status_msg
float32 left_wheel_speed
float32 right_wheel_speed
float32 imu_angular_z
float32 gps_lat
float32 gps_lon