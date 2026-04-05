#include "rtk_interface.h"
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
class RTKInterface {
public:
    RTKInterface(const std::string& port, int baudrate)
        : port_(port), baudrate_(baudrate), serial_port_(new serial::Serial()) {
        // 设置串口参数
        serial_port_->setPort(port_);
        serial_port_->setBaudrate(baudrate_);
        serial_port_->setBytesize(serial::eightbits);
        serial_port_->setParity(serial::parity_none);
        serial_port_->setStopbits(serial::stopbits_one);
        serial_port_->setFlowcontrol(serial::flowcontrol_none);
    }
    
    bool init() {
        try {
            if (!serial_port_->isOpen()) {
                serial_port_->open();
            }
            return serial_port_->isOpen();
        } catch (serial::IOException& e) {
            ROS_ERROR("RTK serial port error: %s", e.what());
            return false;
        }
    }
    
    bool readGPS(sensor_msgs::NavSatFix& gps_msg) {
        // 从串口读取NMEA数据（简化实现）
        std::string line;
        if (serial_port_->available() > 0) {
            line = serial_port_->readline();
            
            // 解析GGA消息（示例）
            if (line.find("$GNGGA") != std::string::npos) {
                std::vector<std::string> tokens = split(line, ',');
                if (tokens.size() >= 15) {
                    // 解析纬度
                    double lat_deg = std::stod(tokens[2].substr(0, 2));
                    double lat_min = std::stod(tokens[2].substr(2));
                    double latitude = lat_deg + lat_min/60.0;
                    if (tokens[3] == "S") latitude = -latitude;
                    
                    // 解析经度
                    double lon_deg = std::stod(tokens[4].substr(0, 3));
                    double lon_min = std::stod(tokens[4].substr(3));
                    double longitude = lon_deg + lon_min/60.0;
                    if (tokens[5] == "W") longitude = -longitude;
                    
                    // 解析海拔
                    double altitude = std::stod(tokens[9]);
                    
                    // 解析定位状态
                    int status = std::stoi(tokens[6]);
                    sensor_msgs::NavSatStatus nav_status;
                    nav_status.status = (status == 0) ? sensor_msgs::NavSatStatus::NO_FIX :
                                         (status == 1) ? sensor_msgs::NavSatStatus::FIX :
                                         (status == 2) ? sensor_msgs::NavSatStatus::DGPS :
                                         sensor_msgs::NavSatStatus::STATUS_ERROR;
                    nav_status.service = sensor_msgs::NavSatStatus::GPS;
                    
                    // 设置GPS消息
                    gps_msg.header.stamp = ros::Time::now();
                    gps_msg.header.frame_id = "gps_link";
                    gps_msg.status = nav_status;
                    gps_msg.latitude = latitude;
                    gps_msg.longitude = longitude;
                    gps_msg.altitude = altitude;
                    
                    // 设置协方差（示例：对角矩阵）
                    gps_msg.position_covariance[0] = 0.5;  // 纬度方向不确定性
                    gps_msg.position_covariance[4] = 0.5;  // 经度方向不确定性
                    gps_msg.position_covariance[8] = 1.0;  // 海拔方向不确定性
                    gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
                    
                    return true;
                }
            }
        }
        return false;
    }
    
private:
    std::string port_;
    int baudrate_;
    std::unique_ptr<serial::Serial> serial_port_;
    
    // 字符串分割函数
    std::vector<std::string> split(const std::string& s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }
};