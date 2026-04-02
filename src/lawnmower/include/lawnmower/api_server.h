#ifndef API_SERVER_H
#define API_SERVER_H

#include <ros/ros.h>
#include <webserver/libhttpserver.h>
#include "srv/SetSensorParam.h"
#include "srv/GetSensorStatus.h"
#include "srv/CalibrateSensor.h"
#include "lawnmower/orbbec_gemini335_driver.h"
#include "lawnmower/hesai_jt128_driver.h"
#include "lawnmower/um982_rtk_driver.h"

namespace lawnmower {

class APIServer {
public:
    APIServer(ros::NodeHandle& nh, 
              OrbbecGemini335Driver* orbbec_driver,
              HesaiJT128Driver* hesai_driver,
              UM982RTKDriver* um982_driver,
              const std::string& config_path);
    ~APIServer();

    void start();
    void stop();

private:
    bool setSensorParamCallback(srv::SetSensorParam::Request& req,
                                srv::SetSensorParam::Response& res);
    bool getSensorStatusCallback(srv::GetSensorStatus::Request& req,
                                 srv::GetSensorStatus::Response& res);
    bool calibrateSensorCallback(srv::CalibrateSensor::Request& req,
                                 srv::CalibrateSensor::Response& res);
    void loadConfig(const std::string& config_path);

    ros::NodeHandle nh_;
    ros::ServiceServer set_param_srv_;
    ros::ServiceServer get_status_srv_;
    ros::ServiceServer calibrate_srv_;

    OrbbecGemini335Driver* orbbec_driver_;
    HesaiJT128Driver* hesai_driver_;
    UM982RTKDriver* um982_driver_;

    int server_port_;
    bool debug_mode_;
    std::string log_level_;
    std::string param_update_topic_;

    httpserver::webserver* http_server_;
};

}
#endif