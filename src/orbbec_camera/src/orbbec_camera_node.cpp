#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "orbbec_camera_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/depth/image_raw",10);
  ros::Rate rate(30);

  sensor_msgs::Image msg;
  msg.header.frame_id = "camera_link";
  msg.width = 640; msg.height=480;
  msg.encoding = "16UC1";
  msg.step = msg.width*2;
  msg.data.resize(msg.step*msg.height,0);

  while(ros::ok()){
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}