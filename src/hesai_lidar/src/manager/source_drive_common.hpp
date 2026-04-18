#pragma once
#include "utility/yaml_reader.hpp"
#include "hesai_lidar_sdk.hpp"
class DriveYamlParam
{
public:
    DriveYamlParam() {};
    ~DriveYamlParam() {};

    bool GetDriveYamlParam(const YAML::Node& config, DriverParam &driver_param)
    {
        YAML::Node driver_config = YamlSubNodeAbort(config, "driver");
        int source_type;

        // input related
        YamlRead<bool>(       driver_config, "use_gpu",                 driver_param.use_gpu, false);
        YamlRead<uint16_t>(   driver_config, "udp_port",                driver_param.input_param.udp_port, 2368);
        YamlRead<uint16_t>(   driver_config, "ptc_port",                driver_param.input_param.ptc_port, 9347);
        YamlRead<std::string>(driver_config, "device_ip_address",       driver_param.input_param.device_ip_address, "");
        YamlRead<std::string>(driver_config, "multicast_ip_address",    driver_param.input_param.multicast_ip_address, "");
        YamlRead<std::string>(driver_config, "pcap_path",               driver_param.input_param.pcap_path, "");
        YamlRead<std::string>(driver_config, "firetimes_path",          driver_param.input_param.firetimes_path, "");
        YamlRead<std::string>(driver_config, "correction_file_path",    driver_param.input_param.correction_file_path, "");
        YamlRead<bool>       (driver_config, "is_use_ptc",              driver_param.input_param.is_use_ptc, false);
        // decoder related
        YamlRead<bool>(       driver_config, "pcap_play_synchronization", driver_param.decoder_param.pcap_play_synchronization, false);
        YamlRead<bool>(       driver_config, "transform_flag",            driver_param.decoder_param.transform_param.use_flag, false);
        YamlRead<float>(      driver_config, "x",                         driver_param.decoder_param.transform_param.x, 0);
        YamlRead<float>(      driver_config, "y",                         driver_param.decoder_param.transform_param.y, 0);
        YamlRead<float>(      driver_config, "z",                         driver_param.decoder_param.transform_param.z, 0);
        YamlRead<float>(      driver_config, "roll",                      driver_param.decoder_param.transform_param.roll, 0);
        YamlRead<float>(      driver_config, "pitch",                     driver_param.decoder_param.transform_param.pitch, 0);
        YamlRead<float>(      driver_config, "yaw",                       driver_param.decoder_param.transform_param.yaw, 0);
        YamlRead<float>(      driver_config, "frame_start_azimuth",       driver_param.decoder_param.frame_start_azimuth, -1);
        YamlRead<uint16_t>(   driver_config, "use_timestamp_type",        driver_param.decoder_param.use_timestamp_type, 0);
        YamlRead<int>(        driver_config, "fov_start",                 driver_param.decoder_param.fov_start, -1);
        YamlRead<int>(        driver_config, "fov_end",                   driver_param.decoder_param.fov_end, -1);
        YamlRead<int>(        driver_config, "source_type",               source_type, 0);
        YamlRead<bool>(       driver_config, "distance_correction_flag",  driver_param.decoder_param.distance_correction_flag, false);
        driver_param.input_param.source_type = SourceType(source_type);
        // ROS related
        YamlRead<bool>(       config["ros"], "send_packet_ros",            driver_param.input_param.send_packet_ros, false);
        YamlRead<bool>(       config["ros"], "send_point_cloud_ros",       driver_param.input_param.send_point_cloud_ros, false);
        YamlRead<std::string>(config["ros"], "ros_frame_id",               driver_param.input_param.frame_id, "hesai_lidar");
        YamlRead<std::string>(config["ros"], "ros_send_packet_topic",      driver_param.input_param.ros_send_packet_topic, "hesai_packets");
        YamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", driver_param.input_param.ros_send_point_topic, "hesai_points");
        YamlRead<std::string>(config["ros"], "ros_recv_packet_topic",      driver_param.input_param.ros_recv_packet_topic, "hesai_packets");
        YamlRead<std::string>(config["ros"], "ros_send_correction_topic",  driver_param.input_param.ros_send_correction_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_recv_correction_topic",  driver_param.input_param.ros_recv_correction_topic, NULL_TOPIC);  
        YamlRead<std::string>(config["ros"], "ros_send_imu_topic",         driver_param.input_param.ros_send_imu_topic, NULL_TOPIC);              
        return true;
    }

    
};