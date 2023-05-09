/**
 * @file multi_eval_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief calculate the distance between agents
 * @version 1.0
 * @date 2023-01-08
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <iostream>
#include "eval_helper/multi_eval.hpp"
// #include "eval_helper/multi_eval_child.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_eval_node");
  ros::NodeHandle nh("~");

  MultiAgentLogger multi_eval;

  /* MultiAgentMinDistance multi_eval; */
  multi_eval.init(nh);

  ros::spin();
  return 0;
}
