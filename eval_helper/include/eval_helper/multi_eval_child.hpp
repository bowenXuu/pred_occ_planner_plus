/**
 * @file multi_eval.hpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2023-01-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef MULTI_EVAL_CHILD_HPP
#define MULTI_EVAL_CHILD_HPP

#include "eval_helper/multi_eval.hpp"

class MultiAgentMinDistance : public MultiAgentLogger {
 protected:
  double global_min_dist_;

 public:
  void init(ros::NodeHandle& nh) {
    nh_ = nh;
    nh_.param("num_agents", num_agents_, 1);
    double pos_query_rate;
    nh_.param("pos_query_rate", pos_query_rate, 50.0);

    traj_recorders_.reserve(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
      traj_recorders_.push_back(std::make_shared<SingleTrajRecorder>(nh_, i));
    }

    pos_query_timer_ = nh_.createTimer(ros::Duration(1.0 / pos_query_rate),
                                       &MultiAgentMinDistance::posQueryCallback, this);
  }

  void posQueryCallback(const ros::TimerEvent& event) {
    std::vector<Eigen::Vector3d> positions;
    positions.resize(num_agents_);
    for (int i = 0; i < num_agents_; i++) {
      geometry_msgs::Point position;
      traj_recorders_[i]->getPosition(position);
      positions[i] << position.x, position.y, position.z;
    }

    double min_dist = 1000.0;
    for (int i = 0; i < num_agents_; i++) {
      for (int j = i + 1; j < num_agents_; j++) {
        double dist = (positions[i] - positions[j]).norm();
        ROS_INFO("distance between agent %d and agent %d is %f", i, j, dist);
        if (dist < min_dist) {
          min_dist = dist;
        }
      }
    }

    ROS_INFO("min distance is %f", min_dist);
    if (min_dist < global_min_dist_) {
      global_min_dist_ = min_dist;
    }
    ROS_INFO("Global min distance is %f", global_min_dist_);
  }
};

#endif  // MULTI_EVAL_HPP
