#include "PointCloud2.h"
#include <string.h>
// 初始化PointCloud2消息
void point_cloud2_init(sensor_msgs::PointCloud2* msg, uint32_t height, uint32_t width, bool is_bigendian, 
                      uint32_t point_step, uint32_t row_step, bool is_dense) {
    msg->header = std_msgs::Header();
    msg->height = height;
    msg->width = width;
    msg->is_bigendian = is_bigendian;
    msg->point_step = point_step;
    msg->row_step = row_step;
    msg->data = (uint8_t*)malloc(row_step * height); // 分配内存
    msg->is_dense = is_dense;
}
// 释放PointCloud2消息内存
void point_cloud2_destroy(sensor_msgs::PointCloud2* msg) {
    free(msg->data);
    msg->data = NULL;
}
// 设置点云数据
void point_cloud2_set_data(sensor_msgs::PointCloud2* msg, const uint8_t* data) {
    memcpy(msg->data, data, msg->row_step * msg->height);
}
// 获取指定点的数据
uint8_t* point_cloud2_get_point(sensor_msgs::PointCloud2* msg, uint32_t row, uint32_t col) {
    if (row >= msg->height || col >= msg->width) return NULL;
    return msg->data + row * msg->row_step + col * msg->point_step;
}