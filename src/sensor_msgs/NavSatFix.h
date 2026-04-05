// sensor_msgs/NavSatFix.h
#ifndef SENSOR_MSGS_NAV_SAT_FIX_H
#define SENSOR_MSGS_NAV_SAT_FIX_H
#include <std_msgs/Header.h>
#include <sensor_msgs/NavSatStatus.h>
namespace sensor_msgs {
class NavSatFix {
public:
    std_msgs::Header header;
    NavSatStatus status;
    double latitude;
    double longitude;
    double altitude;
    double position_covariance[9];
    uint8_t position_covariance_type;
    
    enum CovarianceType {
        COVARIANCE_TYPE_UNKNOWN = 0,
        COVARIANCE_TYPE_APPROXIMATED = 1,
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
        COVARIANCE_TYPE_KNOWN = 3
    };
};
}  // namespace sensor_msgs
#endif

// sensor_msgs/NavSatFix.c
#include "NavSatFix.h"
namespace sensor_msgs {
// 实现GPS数据的常见操作
class NavSatFixHelper {
public:
    // 将度分格式转换为十进制度数
    static double dmsToDecimal(double degrees, double minutes, char hemisphere) {
        double decimal = degrees + minutes / 60.0;
        if (hemisphere == 'S' || hemisphere == 'W') {
            decimal = -decimal;
        }
        return decimal;
    }
    
    // 设置协方差矩阵（对角矩阵）
    static void setDiagonalCovariance(double* covariance, double lat, double lon, double alt) {
        covariance[0] = lat;
        covariance[1] = 0;
        covariance[2] = 0;
        covariance[3] = 0;
        covariance[4] = lon;
        covariance[5] = 0;
        covariance[6] = 0;
        covariance[7] = 0;
        covariance[8] = alt;
    }
};
}  // namespace sensor_msgs