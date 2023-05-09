/**
 * @file noise_maker.cpp
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

#include <iostream>
#include <random>

ros::Publisher noisy_odom_pub_, noisy_pose_pub_;

/* TODO: add noise to ground truth obstacle */

/** noise */
int                              seed_;
double                           mu_x_, mu_y_, mu_z_;
double                           sigma_x_, sigma_y_, sigma_z_;
std::default_random_engine       noise_generator_;
std::normal_distribution<double> dis_x_;
std::normal_distribution<double> dis_y_;
std::normal_distribution<double> dis_z_;

void odomCallback(const nav_msgs::Odometry& msg) {
  nav_msgs::Odometry noisy_odom;

  noisy_odom.header.frame_id = msg.header.frame_id;
  noisy_odom.header.seq      = msg.header.seq;
  noisy_odom.header.stamp    = msg.header.stamp;

  /** generate noise */
  double x = dis_x_(noise_generator_);
  double y = dis_y_(noise_generator_);
  double z = dis_z_(noise_generator_);

  //  ROS_INFO_STREAM("Generate noise: x " << x << " y " << y << " z " << z);

  noisy_odom.pose.pose.position.x = msg.pose.pose.position.x + x;
  noisy_odom.pose.pose.position.y = msg.pose.pose.position.y + y;
  noisy_odom.pose.pose.position.z = msg.pose.pose.position.z + z;

  noisy_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w;
  noisy_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x;
  noisy_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y;
  noisy_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z;

  noisy_odom.twist.twist.linear.x = msg.twist.twist.linear.x;
  noisy_odom.twist.twist.linear.y = msg.twist.twist.linear.y;
  noisy_odom.twist.twist.linear.z = msg.twist.twist.linear.z;

  noisy_odom_pub_.publish(noisy_odom);
}

void poseCallback(const geometry_msgs::PoseStamped& msg) {
  geometry_msgs::PoseStamped noisy_pose;
  noisy_pose.header.seq      = msg.header.seq;
  noisy_pose.header.stamp    = msg.header.stamp;
  noisy_pose.header.frame_id = msg.header.frame_id;

  double x = dis_x_(noise_generator_);
  double y = dis_y_(noise_generator_);
  double z = dis_z_(noise_generator_);

  //  ROS_INFO_STREAM("Generate noise: x " << x << " y " << y << " z " << z);
  noisy_pose.pose.position.x    = msg.pose.position.x + x;
  noisy_pose.pose.position.y    = msg.pose.position.y + y;
  noisy_pose.pose.position.z    = msg.pose.position.z + z;
  noisy_pose.pose.orientation.x = msg.pose.orientation.x;
  noisy_pose.pose.orientation.y = msg.pose.orientation.y;
  noisy_pose.pose.orientation.z = msg.pose.orientation.z;
  noisy_pose.pose.orientation.w = msg.pose.orientation.w;

  noisy_pose_pub_.publish(noisy_pose);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_noise_maker");
  ros::NodeHandle nh("~");

  mu_x_ = mu_y_ = mu_z_ = 0.0;
  /* load std dev from ROS parameter server */
  nh.param("noise/stddev/x", sigma_x_, 0.0);
  nh.param("noise/stddev/y", sigma_y_, 0.0);
  nh.param("noise/stddev/z", sigma_z_, 0.0);
  ROS_INFO("[NOISE MAKER] Odom noise sigma_x_ = %lf", sigma_x_);
  ROS_INFO("[NOISE MAKER] Odom noise sigma_y_ = %lf", sigma_y_);
  ROS_INFO("[NOISE MAKER] Odom noise sigma_z_ = %lf", sigma_z_);

  noise_generator_.seed(time(NULL));
  dis_x_ = std::normal_distribution<double>(mu_x_, sigma_x_);
  dis_y_ = std::normal_distribution<double>(mu_y_, sigma_y_);
  dis_z_ = std::normal_distribution<double>(mu_z_, sigma_z_);

  ros::Subscriber odom_sub =
      nh.subscribe("origin_odom", 1, &odomCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber pose_sub =
      nh.subscribe("origin_pose", 1, &poseCallback, ros::TransportHints().tcpNoDelay());

  noisy_odom_pub_ = nh.advertise<nav_msgs::Odometry>("noisy_odom", 1);
  noisy_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("noisy_pose", 1);

  ros::spin();
}
