// @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2022, Institute of Automatic Control - RWTH Aachen University
// All rights reserved.

// ROS
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <lbrmed_hw_fri/lbrmed_hw_fri.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lbrmed_hw_fri");
  ros::NodeHandle nh;
  // NOTE: Run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // init generic hardware interface
  auto lbrmed = lbrmed_hw_fri::HardwareInterface(nh);
  lbrmed.init();

  ROS_INFO_NAMED("lbrmed_hw_fri", "Starting control loop");
  controller_manager::ControllerManager controller_manager(&lbrmed, nh);

  // Sufficiently high rate (100Hz for Postion, 200Hz for Force mode)
  ros::Rate rate(300);
  while (ros::ok())
  {
    auto elapsed = rate.cycleTime();
    lbrmed.read(elapsed);
    controller_manager.update(ros::Time::now(), elapsed, lbrmed.reset_required());
    lbrmed.write(elapsed);
    rate.sleep();
  }
  return EXIT_SUCCESS;
}