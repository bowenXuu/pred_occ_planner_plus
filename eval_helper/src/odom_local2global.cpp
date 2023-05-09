/**
 * @file odom_local2global.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief generate localization noise
 * @version 1.0
 * @date 2022-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <iostream>
#include <random>

Eigen::Vector3d    position_;
Eigen::Quaterniond orientation_;

ros::Publisher global_odom_pub_, global_pose_pub_;
void           odomCallback(const nav_msgs::Odometry& msg) {
  nav_msgs::Odometry global_odom;

  global_odom.header.frame_id         = msg.header.frame_id;
  global_odom.header.seq              = msg.header.seq;
  global_odom.header.stamp            = msg.header.stamp;
  global_odom.pose.pose.position.x    = msg.pose.pose.position.x + position_.x();
  global_odom.pose.pose.position.y    = msg.pose.pose.position.y + position_.y();
  global_odom.pose.pose.position.z    = msg.pose.pose.position.z + position_.z();
  global_odom.pose.pose.orientation.w = orientation_.w();
  global_odom.pose.pose.orientation.x = orientation_.x();
  global_odom.pose.pose.orientation.y = orientation_.y();
  global_odom.pose.pose.orientation.z = orientation_.z();
}

void poseCallback(const geometry_msgs::PoseStamped& msg) {
  geometry_msgs::PoseStamped global_pose;
  global_pose.header.seq      = msg.header.seq;
  global_pose.header.stamp    = msg.header.stamp;
  global_pose.header.frame_id = msg.header.frame_id;

  global_pose.pose.position.x    = msg.pose.position.x + position_.x();
  global_pose.pose.position.y    = msg.pose.position.y + position_.y();
  global_pose.pose.position.z    = msg.pose.position.z + position_.z();
  global_pose.pose.orientation.x = msg.pose.orientation.x;
  global_pose.pose.orientation.y = msg.pose.orientation.y;
  global_pose.pose.orientation.z = msg.pose.orientation.z;
  global_pose.pose.orientation.w = msg.pose.orientation.w;

  global_pose_pub_.publish(global_pose);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_global_odometry");
  ros::NodeHandle nh("~");

  position_    = Eigen::Vector3d::Zero();
  orientation_ = Eigen::Quaterniond::Identity();
  /* load std dev from ROS parameter server */
  nh.param("init_x", position_.x(), 0.0);
  nh.param("init_y", position_.y(), 0.0);
  nh.param("init_z", position_.z(), 0.0);
  nh.param("init_qw", orientation_.w(), 0.0);
  nh.param("init_qx", orientation_.x(), 0.0);
  nh.param("init_qy", orientation_.y(), 0.0);
  nh.param("init_qz", orientation_.z(), 0.0);

  ROS_INFO("[GlbOdom] Init pose: (%f, %f, %f) (%f, %f, %f, %f)", position_.x(), position_.y(),
           position_.z(), orientation_.w(), orientation_.x(), orientation_.y(), orientation_.z());

  ros::Subscriber odom_sub =
      nh.subscribe("local_odom", 1, &odomCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber pose_sub =
      nh.subscribe("local_pose", 1, &poseCallback, ros::TransportHints().tcpNoDelay());

  global_odom_pub_ = nh.advertise<nav_msgs::Odometry>("global_odom", 1);
  global_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("global_pose", 1);

  ros::spin();
}
