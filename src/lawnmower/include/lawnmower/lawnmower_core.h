#ifndef LAWNMOWER_CORE_H
#define LAWNMOWER_CORE_H

#include <ros/ros.h>
#include "lawnmower/orbbec_gemini335_driver.h"
#include "lawnmower/hesai_jt128_driver.h"
#include "lawnmower/um982_rtk_driver.h"
#include "lawnmower/api_server.h"

namespace lawnmower {

class LawnmowerCore {
public:
    LawnmowerCore(ros::NodeHandle& nh);
    ~LawnmowerCore();

    bool init();
    void start();
    void stop();

private:
    ros::NodeHandle nh_;

    OrbbecGemini335Driver* orbbec_driver_;
    HesaiJT128Driver* hesai_driver_;
    UM982RTKDriver* um982_driver_;
    APIServer* api_server_;

    std::string orbbec_config_path_;
    std::string hesai_config_path_;
    std::string um982_config_path_;
    std::string api_config_path_;
};

}
#endif