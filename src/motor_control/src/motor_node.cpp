#include "motor_control/motor_driver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_control_node");
    ros::NodeHandle nh("~");

    MotorDriver driver(nh);
    if (!driver.init()) return -1;

    driver.run();
    return 0;
}