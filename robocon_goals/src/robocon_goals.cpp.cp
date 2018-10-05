#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <string>
#include <queue>
//#include <vector>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;
int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_goal_publisher");

  ros::NodeHandle n;

  ros::Publisher pub =
      n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 5);
  geometry_msgs::PoseWithCovarianceStamped init_pose;

  init_pose.header.stamp = ros::Time::now();
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.position.x = 0.18;
  init_pose.pose.pose.position.y = 1.08;
  init_pose.pose.pose.position.z = 0;
  init_pose.pose.pose.orientation.w = 1.0;

  MoveBaseClient ac("move_base", true);

  ROS_INFO("Waiting for the move_base action server to come up");
  ac.waitForServer();
  // while (!ac.waitForServer(ros::Duration(5.0))) {
  //  ROS_INFO("Waiting for the move_base action server to come up");
  //}

  pub.publish(init_pose);

  std::queue<move_base_msgs::MoveBaseGoal> goal;
  move_base_msgs::MoveBaseGoal goal_tmp;

  goal_tmp.target_pose.header.frame_id = "base_link";
  goal_tmp.target_pose.pose.orientation.w = 1.0;
  goal_tmp.target_pose.header.stamp = ros::Time::now();
  goal_tmp.target_pose.pose.position.x = 2.2;
  goal_tmp.target_pose.pose.position.y = 3.5;
  goal.push(goal_tmp);

  goal_tmp.target_pose.header.stamp = ros::Time::now();
  goal_tmp.target_pose.pose.position.x = 1.0;
  goal_tmp.target_pose.pose.position.y = 1.0;
  goal.push(goal_tmp);
  goal_tmp.target_pose.header.stamp = ros::Time::now();
  goal_tmp.target_pose.pose.position.x = 4.0;
  goal_tmp.target_pose.pose.position.y = 4.0;
  goal.push(goal_tmp);

  std::cout << goal.front().target_pose.pose.position.x << " + " << goal.front().target_pose.pose.position.y << std::endl;
  goal.pop();
  std::cout << goal.front().target_pose.pose.position.x << " + " << goal.front().target_pose.pose.position.y << std::endl;
  goal.pop();
  std::cout << goal.front().target_pose.pose.position.x << " + " << goal.front().target_pose.pose.position.y << std::endl;
  goal.pop();

  while(!goal.empty()){
    ROS_INFO("sending goal");
    ac.sendGoal(goal.front());
    goal.pop();

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Gooooal!!");
    } else {
      ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
  }

  return 0;
}
