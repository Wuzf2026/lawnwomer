#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "handsfree_rtk_node");
  ros::NodeHandle nh;

  ros::Publisher pub_fix = nh.advertise<sensor_msgs::NavSatFix>("/fix",10);
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_rtk",10);
  tf2_ros::TransformBroadcaster br;
  ros::Rate rate(5);

  while(ros::ok()){
    sensor_msgs::NavSatFix fix;
    fix.header.stamp = ros::Time::now();
    fix.header.frame_id = "rtk_link";
    fix.latitude = 30.0;
    fix.longitude = 120.0;
    fix.altitude = 50.0;
    pub_fix.publish(fix);

    nav_msgs::Odometry odom;
    odom.header = fix.header;
    odom.child_frame_id = "base_link";
    odom.pose.pose.orientation.w = 1.0;
    pub_odom.publish(odom);

    geometry_msgs::TransformStamped ts;
    ts.header = odom.header;
    ts.child_frame_id = "base_link";
    ts.transform.rotation.w = 1.0;
    br.sendTransform(ts);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}