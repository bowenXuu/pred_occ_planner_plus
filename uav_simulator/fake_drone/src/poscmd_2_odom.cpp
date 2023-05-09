#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <random>

#include "quadrotor_msgs/PositionCommand.h"

ros::Subscriber _cmd_sub;
ros::Publisher  _odom_pub;
ros::Publisher  _pose_pub;

quadrotor_msgs::PositionCommand _cmd;
double                          _init_x, _init_y, _init_z;
double                          _init_qx, _init_qy, _init_qz, _init_qw;

bool rcv_cmd = false;
void rcvPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd) {
  rcv_cmd = true;
  _cmd    = cmd;
}

void pubOdom() {
  /* odometry headers */
  nav_msgs::Odometry odom;
  odom.header.stamp    = ros::Time::now();
  odom.header.frame_id = "world";

  /* pose headers */
  geometry_msgs::PoseStamped pose;
  pose.header.stamp    = ros::Time::now();
  pose.header.frame_id = "world";

  if (rcv_cmd) {
    odom.pose.pose.position.x = _cmd.position.x;
    odom.pose.pose.position.y = _cmd.position.y;
    odom.pose.pose.position.z = _cmd.position.z;

    Eigen::Vector3d alpha =
        Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y, _cmd.acceleration.z) +
        9.8 * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
    Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
    Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
    Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
    Eigen::Vector3d zB = xB.cross(yB);
    Eigen::Matrix3d R;
    R.col(0) = xB;
    R.col(1) = yB;
    R.col(2) = zB;
    Eigen::Quaterniond q(R);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    odom.twist.twist.linear.x = _cmd.velocity.x;
    odom.twist.twist.linear.y = _cmd.velocity.y;
    odom.twist.twist.linear.z = _cmd.velocity.z;

    odom.twist.twist.angular.x = _cmd.acceleration.x;
    odom.twist.twist.angular.y = _cmd.acceleration.y;
    odom.twist.twist.angular.z = _cmd.acceleration.z;

    /* update pose */
    pose.pose.position.x    = _cmd.position.x;
    pose.pose.position.y    = _cmd.position.y;
    pose.pose.position.z    = _cmd.position.z;
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();

  } else {
    odom.pose.pose.position.x = _init_x;
    odom.pose.pose.position.y = _init_y;
    odom.pose.pose.position.z = _init_z;

    odom.pose.pose.orientation.w = _init_qw;
    odom.pose.pose.orientation.x = _init_qx;
    odom.pose.pose.orientation.y = _init_qy;
    odom.pose.pose.orientation.z = _init_qz;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    /* update pose */
    pose.pose.position.x    = _init_x;
    pose.pose.position.y    = _init_y;
    pose.pose.position.z    = _init_z;
    pose.pose.orientation.w = 1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
  }

  _odom_pub.publish(odom);
  _pose_pub.publish(pose);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_generator");
  ros::NodeHandle nh("~");

  nh.param("init_x", _init_x, 0.0);
  nh.param("init_y", _init_y, 0.0);
  nh.param("init_z", _init_z, 0.0);
  nh.param("init_qx", _init_qx, 0.0);
  nh.param("init_qy", _init_qy, 0.0);
  nh.param("init_qz", _init_qz, 0.0);
  nh.param("init_qw", _init_qw, 1.0);
  _init_qw = (_init_qw > 1.0) ? 1.0 : _init_qw;
  _init_qx = (_init_qx > 1.0) ? 1.0 : _init_qx;
  _init_qy = (_init_qy > 1.0) ? 1.0 : _init_qy;
  _init_qz = (_init_qz > 1.0) ? 1.0 : _init_qz;

  _cmd_sub  = nh.subscribe("command", 1, rcvPosCmdCallBack);
  _odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
  _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

  ros::Rate rate(100);
  bool      status = ros::ok();
  while (status) {
    pubOdom();
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
