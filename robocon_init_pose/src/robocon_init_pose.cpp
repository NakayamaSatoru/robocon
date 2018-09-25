#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robocon_init_pose_publisher");
  ros::NodeHandle n;

  ros::Publisher pub =
      n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 50);
  geometry_msgs::PoseWithCovarianceStamped init_pose;

  init_pose.header.stamp = ros::Time::now();
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.position.x = 0.6;
  init_pose.pose.pose.position.y = 1.0;
  init_pose.pose.pose.position.z = 0;
  init_pose.pose.pose.orientation.w = 1.0;

  pub.publish(init_pose);
}
