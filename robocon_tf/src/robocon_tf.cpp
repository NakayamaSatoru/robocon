#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robocon_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100.0);

  tf::TransformBroadcaster broadcaster;

  while (n.ok()) {
    broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.42, 0.0, 0.12)),
        ros::Time::now(), "base_link", "base_laser"));
    broadcaster.sendTransform(
        tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(),
                             "base_footprint", "base_link"));
    /*
    broadcaster.sendTransform(
        tf::StampedTransform(tf::Transform::getIdentity(), ros::Time::now(),
                             "odom","base_footprint"));
    */
    r.sleep();
  }
}