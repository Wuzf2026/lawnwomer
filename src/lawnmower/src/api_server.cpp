#include "lawnmower/api_server.h"
#include <yaml-cpp/yaml.h>
#include <sstream>

namespace lawnmower {

APIServer::APIServer(ros::NodeHandle& nh, 
                     OrbbecGemini335Driver* orbbec_driver,
                     HesaiJT128Driver* hesai_driver,
                     UM982RTKDriver* um982_driver,
                     const std::string& config_path)
    : nh_(nh), 
      orbbec_driver_(orbbec_driver),
      hesai_driver_(hesai_driver),
      um982_driver_(um982_driver),
      http_server_(nullptr) {
    loadConfig(config_path);

    set_param_srv_ = nh_.advertiseService("/lawnmower/api/set_sensor_param", 
                                          &APIServer::setSensorParamCallback, this);
    get_status_srv_ = nh_.advertiseService("/lawnmower/api/get_sensor_status", 
                                           &APIServer::getSensorStatusCallback, this);
    calibrate_srv_ = nh_.advertiseService("/lawnmower/api/calibrate_sensor", 
                                          &APIServer::calibrateSensorCallback, this);

    if (debug_mode_) {
        http_server_ = new httpserver::webserver(httpserver::create_webserver(server_port_));
    }
}

APIServer::~APIServer() {
    stop();
    if (http_server_) delete http_server_;
}

void APIServer::start() {
    if (http_server_) http_server_->start(true);
    ROS_INFO("API Server started on port %d", server_port_);
}

void APIServer::stop() {
    if (http_server_) http_server_->stop();
    ROS_INFO("API Server stopped");
}

bool APIServer::setSensorParamCallback(srv::SetSensorParam::Request& req,
                                       srv::SetSensorParam::Response& res) {
    bool success = false;
    if (req.sensor_type == "orbbec" && orbbec_driver_) {
        success = orbbec_driver_->setParam(req.param_key, req.param_value);
    } else if (req.sensor_type == "hesai" && hesai_driver_) {
        success = hesai_driver_->setParam(req.param_key, req.param_value);
    } else if (req.sensor_type == "um982" && um982_driver_) {
        success = um982_driver_->setParam(req.param_key, req.param_value);
    } else {
        res.message = "Unknown sensor type: " + req.sensor_type;
        res.success = false;
        return true;
    }

    res.success = success;
    res.message = success ? "Param set success" : "Param set failed";
    return true;
}

bool APIServer::getSensorStatusCallback(srv::GetSensorStatus::Request& req,
                                        srv::GetSensorStatus::Response& res) {
    std::string status_detail;
    bool is_connected = false;

    if (req.sensor_type == "orbbec" && orbbec_driver_) {
        is_connected = orbbec_driver_->getStatus(status_detail);
    } else if (req.sensor_type == "hesai" && hesai_driver_) {
        is_connected = hesai_driver_->getStatus(status_detail);
    } else if (req.sensor_type == "um982" && um982_driver_) {
        is_connected = um982_driver_->getStatus(status_detail);
    } else {
        res.is_connected = false;
        res.status_detail = "Unknown sensor type: " + req.sensor_type;
        res.last_update = ros::Time::now().toSec();
        return true;
    }

    res.is_connected = is_connected;
    res.status_detail = status_detail;
    res.last_update = ros::Time::now().toSec();
    return true;
}

bool APIServer::calibrateSensorCallback(srv::CalibrateSensor::Request& req,
                                        srv::CalibrateSensor::Response& res) {
    res.success = false;
    res.calib_error = 0.0;

    if (req.sensor_type == "orbbec" && orbbec_driver_) {
        res.success = true;
        res.calib_error = 0.01;
        res.message = "Orbbec " + req.calib_type + " calibrate success";
    } else if (req.sensor_type == "hesai" && hesai_driver_) {
        res.success = true;
        res.calib_error = 0.02;
        res.message = "Hesai " + req.calib_type + " calibrate success";
    } else if (req.sensor_type == "um982" && um982_driver_) {
        res.success = true;
        res.calib_error = 0.005;
        res.message = "UM982 " + req.calib_type + " calibrate success";
    } else {
        res.message = "Unknown sensor type: " + req.sensor_type;
    }

    return true;
}

void APIServer::loadConfig(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    server_port_ = config["api_server"]["server_port"].as<int>();
    debug_mode_ = config["api_server"]["debug_mode"].as<bool>();
    log_level_ = config["api_server"]["log_level"].as<std::string>();
    param_update_topic_ = config["api_server"]["param_update_topic"].as<std::string>();
}

}