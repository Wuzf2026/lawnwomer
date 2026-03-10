#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class Lawnwomer {
public:
  Lawnwomer() {
    scan_sub_ = nh_.subscribe("/points_raw", 1, &Lawnwomer::scanCB, this);
    cmd_pub_ = nh_.advertise<geometry_msgs/Twist>("/cmd_vel", 10);
  }

  void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan) {
    geometry_msgs::Twist cmd;
    bool obstacle = false;
    for (float r : scan->ranges) {
      if (r > 0.1 && r < 0.5) { obstacle = true; break; }
    }
    if (obstacle) { cmd.angular.z = 0.6; cmd.linear.x = 0.0; }
    else { cmd.linear.x = 0.3; cmd.angular.z = 0.0; }
    cmd_pub_.publish(cmd);
  }
private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lawnwomer_node");
  Lawnwomer robot;
  ros::spin();
  return 0;
}