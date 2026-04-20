#include "lawnmower_base/rtk_driver.h"

RTKDriver::RTKDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      serial_port_(nullptr),
      new_gps_data_(false)
{
    // 串口USB参数读取
    nh_private_.param<std::string>("uart_dev", uart_dev_, "/dev/ttyUSB0");
    nh_private_.param<int>("baudrate", baudrate_, 115200);
    nh_private_.param<std::string>("gps_frame_id", gps_frame_id_, "um982_gps_link");
}

bool RTKDriver::init()
{
    // 初始化handsfree_rtk UM982串口驱动
    try
    {
        serial_port_ = new serial::Serial(uart_dev_, baudrate_, serial::Timeout::simpleTimeout(100));
        if(!serial_port_->available())
        {
            ROS_ERROR("UM982 USB uart device not connect");
            return false;
        }

        // RTK差分定位解码实例初始化
        rtk_parser_.InitUM982Decode();
        ROS_INFO("UM982 RTK usb uart driver init success");
    }
    catch(std::exception& e)
    {
        ROS_ERROR("RTK serial init err:%s",e.what());
        return false;
    }

    // ROS标准NavSatFix GPS定位话题发布
    pub_gps_fix_ = nh_.advertise<sensor_msgs::NavSatFix>("/um982/gps_fix", 10);
    pub_nmea_raw_ = nh_.advertise<std_msgs::String>("/um982/nmea_raw", 10);

    return true;
}

void RTKDriver::readData()
{
    if(!serial_port_) return;

    uint8_t buf[1024];
    size_t len = serial_port_->read(buf,1024);
    if(len <=0) return;

    // NMEA原始报文解析
    std::string nmea_str((char*)buf,len);
    std_msgs::String nmea_msg;
    nmea_msg.data = nmea_str;
    pub_nmea_raw_.publish(nmea_msg);

    // UM982定位信息解析
    RtkPosInfo pos_info;
    bool decode_ok = rtk_parser_.ParseNmea(buf,len,pos_info);

    if(decode_ok && pos_info.fix_mode >=2)
    {
        sensor_msgs::NavSatFixPtr gps_msg(new sensor_msgs::NavSatFix);

        gps_msg->header.stamp = ros::Time::now();
        gps_msg->header.frame_id = gps_frame_id_;

        // GPS状态位
        gps_msg->status.status = pos_info.fix_mode;
        gps_msg->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        // 经纬度海拔
        gps_msg->latitude = pos_info.lat;
        gps_msg->longitude = pos_info.lon;
        gps_msg->altitude = pos_info.alt;

        // 位置协方差
        memset(gps_msg->position_covariance.data(),0,72);
        gps_msg->position_covariance[0] = pos_info.hacc;
        gps_msg->position_covariance[4] = pos_info.vacc;
        gps_msg->position_covariance[8] = pos_info.acc;
        gps_msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gps_fix_msg_ = gps_msg;
        new_gps_data_ = true;
        pub_gps_fix_.publish(gps_msg);
    }
}