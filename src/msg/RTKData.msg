# UM982 RTK定位数据
time stamp
float64 latitude   # 纬度 (°)
float64 longitude  # 经度 (°)
float64 altitude   # 海拔 (m)
float32 heading    # 航向角 (°)
uint8 fix_status   # 定位状态：0-未定位，1-单点定位，2-RTK固定解
float32 hdop       # 水平精度因子