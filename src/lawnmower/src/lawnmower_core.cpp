#include "lawnmower/lawnmower_core.h"
#include <ros/package.h>

namespace lawnmower {

LawnmowerCore::LawnmowerCore(ros::NodeHandle& nh)
    : nh_(nh),
      orbbec_driver_(nullptr),
      hesai_driver_(nullptr),
      um982_driver_(nullptr),
      api_server_(nullptr) {
    nh_.param<std::string>("orbbec_config_path", orbbec_config_path_, 
                           ros::package::getPath("config") + "/config/orbbec_config.yaml");
    nh_.param<std::string>("hesai_config_path", hesai_config_path_, 
                           ros::package::getPath("config") + "/config/hesai_config.yaml");
    nh_.param<std::string>("um982_config_path", um982_config_path_, 
                           ros::package::getPath("config") + "/config/um982_config.yaml");
    nh_.param<std::string>("api_config_path", api_config_path_, 
                           ros::package::getPath("config") + "/config/api_config.yaml");
}

LawnmowerCore::~LawnmowerCore() {
    stop();
    if (api_server_) delete api_server_;
    if (um982_driver_) delete um982_driver_;
    if (hesai_driver_) delete hesai_driver_;
    if (orbbec_driver_) delete orbbec_driver_;
}

bool LawnmowerCore::init() {
    orbbec_driver_ = new OrbbecGemini335Driver(nh_, orbbec_config_path_);
    if (!orbbec_driver_->init()) {
        ROS_ERROR("Orbbec driver init failed");
        return false;
    }

    hesai_driver_ = new HesaiJT128Driver(nh_, hesai_config_path_);
    if (!hesai_driver_->init()) {
        ROS_ERROR("Hesai driver init failed");
        return false;
    }

    um982_driver_ = new UM982RTKDriver(nh_, um982_config_path_);
    if (!um982_driver_->init()) {
        ROS_ERROR("UM982 driver init failed");
        return false;
    }

    api_server_ = new APIServer(nh_, orbbec_driver_, hesai_driver_, um982_driver_, api_config_path_);
    if (!api_server_) {
        ROS_ERROR("API server init failed");
        return false;
    }

    ROS_INFO("Lawnmower core init success");
    return true;
}

void LawnmowerCore::start() {
    orbbec_driver_->startCapture();
    hesai_driver_->startCapture();
    um982_driver_->startCapture();
    api_server_->start();
    ROS_INFO("Lawnmower core started");
}

void LawnmowerCore::stop() {
    if (api_server_) api_server_->stop();
    if (um982_driver_) um982_driver_->stopCapture();
    if (hesai_driver_) hesai_driver_->stopCapture();
    if (orbbec_driver_) orbbec_driver_->stopCapture();
    ROS_INFO("Lawnmower core stopped");
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lawnmower_core_node");
    ros::NodeHandle nh;

    lawnmower::LawnmowerCore core(nh);
    if (!core.init()) {
        ROS_FATAL("Lawnmower core init failed, exit");
        return -1;
    }

    core.start();
    ros::spin();
    core.stop();
    return 0;
}