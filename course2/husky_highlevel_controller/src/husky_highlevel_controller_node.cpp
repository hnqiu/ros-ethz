/* Copyright (C) 2019-2020 hnqiu. All Rights Reserved.
 * Licensed under the BSD-3-Clause license. See LICENSE for details.
 */


#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);

  ros::spin();
  return 0;
}
