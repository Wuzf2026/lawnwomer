#ifndef SENSOR_MSGS_POINT_CLOUD2_H
#define SENSOR_MSGS_POINT_CLOUD2_H
#include <std_msgs/Header.h>
#include <stdint.h>
namespace sensor_msgs {
struct PointCloud2 {
    std_msgs::Header header;
    uint32_t height;
    uint32_t width;
    bool is_bigendian;
    uint32_t point_step;
    uint32_t row_step;
    uint8_t* data;
    bool is_dense;
};
} // namespace sensor_msgs
#endif // SENSOR_MSGS_POINT_CLOUD2_H