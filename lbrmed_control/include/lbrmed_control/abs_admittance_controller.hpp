// @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
// All rights reserved.

#pragma once

#include <actionlib/server/simple_action_server.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <lbrmed_msgs/AbsAdmittanceControlAction.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

namespace lbrmed_control
{
/**
 * Admittance controller which keeps moving in positive z direction with a constant velocity until the desired force
 * limit is reached.
 * The absolute force is the euclidean norm of the difference of the initial tare force and the currently measured force
 *
 * norm2(tare - measured) < force_limit
 */
class AbsAdmittanceController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  AbsAdmittanceController()
  {
  }
  ~AbsAdmittanceController()
  {
  }

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  /**
   * \brief Compute the wrench in the base frame
   */
  Eigen::Matrix<double, 6, 1> wrench_in_base(const KDL::JntArrayAcc& joint_states);

  /**
   * \brief Get the current joint positions, velocities, and efforts as KDL array
   *
   * Efforts are stored instead of accelerations
   */
  KDL::JntArrayAcc get_joint_states();

  void goal_action_callback();
  void preempt_action_callback();

  void publish_6d_wrenches(Eigen::Matrix<double, 6, 1> measured_wrench, ros::Time stamp);

  // Safety limit which should not be exceeded even when using the static tare
  double force_limit_;
  // Time in motion after which the dynamic tare force is set
  double dynamic_tare_time_;

  // Values of the action goal, do not move initially
  double goal_abs_force_ = 0.0;
  double velocity_ = 0.0;

  // Kinematics
  std::vector<hardware_interface::JointHandle> joints_;
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::string base_link_;

  // Static gravitational force of tool
  Eigen::Vector3d static_tare_force_;
  // Measured tool force changes due to dynamic friction instead of static
  Eigen::Vector3d dynamic_tare_force_;
  bool dynamic_tare_set_;
  // Integrate the velocity from this position for absolute positioning without drift
  KDL::Frame start_pose_;
  // Time spent in motion, i.e., when force is below the limit
  double motion_seconds_;

  // Accept external goals
  std::unique_ptr<actionlib::SimpleActionServer<lbrmed_msgs::AbsAdmittanceControlAction>> action_server_;

  // Send 6D wrenches as raw results for reproducibility
  ros::Publisher wrench_publisher_;
  ros::Publisher tare_publisher_;
};
}  // namespace lbrmed_control
