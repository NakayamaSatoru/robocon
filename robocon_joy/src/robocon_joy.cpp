#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

double x = 0, y = 0;
void callBack(const sensor_msgs::Joy::ConstPtr &joy) {
  x = joy->axes[0] * (-1);
  y = joy->axes[1];
  ROS_INFO("%f,%f", x, y);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "robocon_joy_publisher");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 50);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &callBack);
  ros::Rate loop(50);
  geometry_msgs::Twist twist;
  while (ros::ok()) {
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z  = 0.0;
    pub.publish(twist);
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
