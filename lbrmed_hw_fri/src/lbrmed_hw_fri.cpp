// @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2022, Institute of Automatic Control - RWTH Aachen University
// All rights reserved.

// Project
#include <cstddef>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <lbrmed_hw_fri/lbrmed_hw_fri.hpp>
// C++
#include <algorithm>
#include <cmath>
#include <memory>
// Pluginlib export for lbrmed_hw_fri
#include <pluginlib/class_list_macros.hpp>
// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
// ROS
#include <ros/console.h>
#include <ros_control_boilerplate/combinable_generic_hw.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <transmission_interface/transmission_interface.h>
// KUKA lib
#include <fri/friLBRState.h>

namespace lbrmed_hw_fri
{
HardwareInterface::HardwareInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : GenericHWInterface(nh, urdf_model), fri_connection(1000)
{
  // Load params
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("arm_id", name_, "lbrmed14");
  private_nh.param<int>("port", port, 30200);
  size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh, "end_effector_link", ee_link);
  error += !rosparam_shortcuts::get(name_, nh, "base_link", base_link);
  error += !rosparam_shortcuts::get(name_, nh, "effort_pt1_T", effort_pt1_T);
  error += !rosparam_shortcuts::get(name_, nh, "velocity_pt1_T", velocity_pt1_T);
  rosparam_shortcuts::shutdownIfError(name_, error);
  // Build the FRI objects
  unfiltered_effort.resize(7);
  ROS_INFO_NAMED(name_, "Setting up FRI Client Application ");
  fri_client = std::make_unique<LBRMedFRIClient>(name_, joint_position_, unfiltered_effort, joint_position_command_,
                                                 joint_effort_command_);
  fri_app = std::make_unique<KUKA::FRI::ClientApplication>(fri_connection, *fri_client);
}

HardwareInterface::~HardwareInterface()
{
  fri_app->disconnect();
}

void HardwareInterface::init()
{
  ROS_INFO_NAMED(name_, "Initializing LBRMed FRI HardwareInterface.");
  // initializes URDF and buffers
  use_soft_limits_if_available_ = true;
  GenericHWInterface::init();

  //  Compare number of joints from the config with the ones reported by the FRI lib
  if (joint_names_.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Number of joints must be " << KUKA::FRI::LBRState::NUMBER_OF_JOINTS << ", got "
                                                              << joint_names_.size());
  }
  else
  {
    ROS_INFO_STREAM_NAMED(name_, "Initialized " << joint_names_.size() << " joints.");
  }

  // Initialize the jacobian solver
  ROS_INFO_NAMED(name_, "Initializing the jacobian solver.");
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, tree))
  {
    const std::string error = "Failed to parse KDL tree from urdf model";
    ROS_ERROR_STREAM_NAMED(name_, error);
    throw std::runtime_error(error);
  }
  if (!tree.getChain(base_link, ee_link, chain))
  {
    const std::string error = ""
                              "Failed to parse robot chain from urdf model. "
                              "Are you sure that both your 'base_link' and 'end_effector_link' exist?";
    ROS_ERROR_STREAM_NAMED(name_, error);
    throw std::runtime_error(error);
  }
  ROS_INFO_STREAM_NAMED(name_, "Initialized chain with " << chain.getNrOfJoints() << " joints and "
                                                         << chain.getNrOfSegments() << " segments");
  // chain lifetime must be the same as jac_solver
  jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(chain);
  jacobian.resize(chain.getNrOfJoints());
  jacobian.data.setZero();
  joint_positions_kdl.resize(chain.getNrOfJoints());
  joint_positions_kdl.data.setZero();
  ROS_INFO_NAMED(name_, "Jacobian solver initialized.");

  // Initialize Force Torque interface
  ROS_INFO_NAMED(name_, "Initializing force torque Interface.");
  hardware_interface::ForceTorqueSensorHandle ft_handle("lbrmed_ft", base_link, force.data(), torque.data());
  ft_interface.registerHandle(ft_handle);
  registerInterface(&ft_interface);
  ROS_INFO_NAMED(name_, "Force torque interface initialized.");

  ROS_INFO_NAMED(name_, "LBRMed FRI HardwareInterface initialized.");
}

void HardwareInterface::read(ros::Duration& elapsed_time)
{
  // avoid setting values at shutdown
  if (!reconnect() || !ros::ok())
  {
    return;
  }
  // Fetch latest data from FRI, will not command values in first step which is MONITORING
  if (!fri_app->step())
  {
    fri_app->disconnect();
  }
  // connectivity check
  if (!fri_connection.isOpen())
  {
    return;
  }

  // Convert to seconds for low pass filter
  auto elapsed_sec = elapsed_time.toSec();
  // Torque sensors are noisy, low pass filter
  for (int i = 0; i < joint_effort_.size(); i++)
  {
    joint_effort_[i] = low_pass_filter(joint_effort_[i], unfiltered_effort[i], effort_pt1_T, elapsed_sec);
  }
  // Velocity via finite difference and low pass filter, velocities not supported in FRI
  // Avoid division by zero
  if (elapsed_sec > 0)
  {
    for (int i = 0; i < joint_velocity_.size(); i++)
    {
      double vel = (joint_position_[i] - last_position_[i]) / elapsed_sec;
      joint_velocity_[i] = low_pass_filter(joint_velocity_[i], vel, velocity_pt1_T, elapsed_sec);
    }
  }
  std::copy(joint_position_.begin(), joint_position_.end(), last_position_.begin());
  // Read torques and calculate external force, forces not supported in FRI
  std::copy(joint_position_.begin(), joint_position_.end(), joint_positions_kdl.data.data());
  auto joint_effort_eigen = Eigen::Map<Eigen::Matrix<double, 7, 1>>(joint_effort_.data());
  jac_solver->JntToJac(joint_positions_kdl, jacobian);
  Eigen::Matrix<double, 6, 1> force_torque_sol =
      jacobian.data.transpose().completeOrthogonalDecomposition().solve(joint_effort_eigen);
  std::copy(force_torque_sol.data(), force_torque_sol.data() + 3, force.data());
  std::copy(force_torque_sol.data() + 3, force_torque_sol.data() + 6, torque.data());
}

void HardwareInterface::write(ros::Duration& elapsed_time)
{
  if (reset_required())
  {
    // Reset safety controllers, old states might cause jerky motions
    reset();
    joint_position_command_.assign(joint_position_.begin(), joint_position_.end());
  }
  if (fri_connection.isOpen() && !ros::ok())
  {
    // Apply safety control
    enforceLimits(elapsed_time);
  }
}

bool HardwareInterface::reset_required()
{
  return fri_client->get_reset_required();
}

void HardwareInterface::enforceLimits(ros::Duration& elapsed_time)
{
  // Do not enforce limits, if no current data is available
  pos_jnt_sat_interface_.enforceLimits(elapsed_time);
  pos_jnt_soft_limits_.enforceLimits(elapsed_time);
}

bool HardwareInterface::reconnect()
{
  if (fri_connection.isOpen())
  {
    return true;
  }

  ROS_INFO_STREAM_NAMED(name_, "Setting up KUKA FRI UDP socket, port: " << port);
  // Connect FRI: Parameter NULL means: repeat to the address where data came from
  if (!ros::ok() || !fri_app->connect(port, nullptr))
  {
    ROS_WARN_NAMED(name_, "Failed to setup FRI UPD socker");
    return false;
  }
  ROS_INFO_STREAM_NAMED(name_, "Successfully set up KUKA FRI UDP socket, port: " << port);
  return true;
}

double HardwareInterface::low_pass_filter(double y_k, double u_k, double T, double d_t)
{
  if (T > d_t)
  {
    auto alpha = d_t / (T + d_t);
    return alpha * u_k + (1 - alpha) * y_k;
  }
  else
  {
    // Disable filtering if time constant is too small
    return u_k;
  }
}

}  // namespace lbrmed_hw_fri
PLUGINLIB_EXPORT_CLASS(ros_control_boilerplate::CombinableGenericHW<lbrmed_hw_fri::HardwareInterface>,
                       hardware_interface::RobotHW)