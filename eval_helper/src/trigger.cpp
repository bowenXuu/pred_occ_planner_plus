/**
 * @file trigger.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-02-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "trigger");
  ros::NodeHandle nh("~");

  /** trigger: random pose */
  geometry_msgs::PoseStamped ps;
  ps.header.seq         = 0;
  ps.header.frame_id    = "world";
  ps.header.stamp       = ros::Time::now();
  ps.pose.orientation.w = 1;
  ps.pose.orientation.x = 0;
  ps.pose.orientation.y = 0;
  ps.pose.orientation.z = 0;
  ps.pose.position.x    = 1;
  ps.pose.position.y    = 1;
  ps.pose.position.z    = 0;

  ros::Publisher trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 1);

  for (int i = 0; i < 40; i++) {
    trigger_pub.publish(ps);
    ros::Duration(0.01).sleep();
  }

  return 0;
}
