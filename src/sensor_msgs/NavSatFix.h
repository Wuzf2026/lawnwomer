#ifndef SENSOR_MSGS_NAV_SAT_FIX_H
#define SENSOR_MSGS_NAV_SAT_FIX_H
#include <std_msgs/Header.h>
namespace sensor_msgs {
struct NavSatFix {
    std_msgs::Header header;
    double latitude;
    double longitude;
    double altitude;
    double position_covariance[9];
    uint8_t position_covariance_type;
};
} // namespace sensor_msgs
#endif // SENSOR_MSGS_NAV_SAT_FIX_H