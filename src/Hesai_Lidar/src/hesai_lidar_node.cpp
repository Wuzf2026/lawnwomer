#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <random>

int main(int argc, char** argv) {
  ros::init(argc, argv, "hesai_lidar_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/points_raw", 10);
  ros::Rate rate(10);

  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "laser_link";
  msg.height = 1;
  msg.width = 1000;
  msg.fields.resize(3);
  msg.fields[0].name = "x"; msg.fields[0].offset = 0; msg.fields[0].datatype = 7;
  msg.fields[1].name = "y"; msg.fields[1].offset = 4; msg.fields[1].datatype = 7;
  msg.fields[2].name = "z"; msg.fields[2].offset = 8; msg.fields[2].datatype = 7;
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> d(-5,5);

  while (ros::ok()) {
    msg.header.stamp = ros::Time::now();
    for (int i=0; i<1000; i++) {
      float* p = (float*)(msg.data.data() + i*16);
      p[0] = d(gen); p[1] = d(gen); p[2] = d(gen);
    }
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}