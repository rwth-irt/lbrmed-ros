// @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2022, Institute of Automatic Control - RWTH Aachen University
// All rights reserved.

#pragma once

// stdlib
#include <array>
#include <memory>
// KDL
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
// ROS
#include <hardware_interface/force_torque_sensor_interface.h>
#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <urdf/model.h>
// KUKA FRI
#include <fri/friClientApplication.h>
#include <fri/friLBRState.h>
#include <fri/friUdpConnection.h>
// FRI Hardware Interface
#include <lbrmed_hw_fri/lbrmed_fri_client.hpp>

namespace lbrmed_hw_fri
{
/**
 * LBRMed FRI interface for ros_control.
 */
class HardwareInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /** \brief Constructor to setup the hardware topics and config
   *
   * \param nh NodeHandle for topics.
   * \param urdf_model - optional pointer to a parsed robot model
   */
  HardwareInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /**
   * \brief Destructor
   */
  ~HardwareInterface();

  /** \brief The init function is called to initialize the RobotHW from a
   * non-realtime thread.
   */
  void init() override;

  /** \brief Read the state from the robot hardware. */
  void read(ros::Duration& elapsed_time) override;

  /** \brief Write the command to the robot hardware. */
  void write(ros::Duration& elapsed_time) override;

  /** \brief returns true if the controllers need to be reset after a change in the FRI connection. */
  bool reset_required();

  /** \brief Enforce limits for all values before writing */
  void enforceLimits(ros::Duration& elapsed_time) override;

private:
  /** \brief Possible hardware interfaces */
  enum InterfaceEnum
  {
    NONE,
    EFFORT,
    POSITION
  };

  /**
   * \brief If not connected, tries to reconnect to the Sunrise controller.
   * \returns True if connected false if not
   */
  bool reconnect();

  /**
   * \brief Time discrete low pass filter using forward differences. No gain is used, since we want to converge on the
   * input value.
   * \param y_k Previous value
   * \param u_k New measured value
   * \param T Time constant
   * \param d_t Elapsed time
   */
  double low_pass_filter(double y_k, double u_k, double T, double d_t);

  // Realtime connection
  int port;
  std::unique_ptr<LBRMedFRIClient> fri_client;
  KUKA::FRI::UdpConnection fri_connection;
  std::unique_ptr<KUKA::FRI::ClientApplication> fri_app;

  // For finite difference approximation of velocity
  std::array<double, 7> last_position_;

  // Force torque from external joint torques
  hardware_interface::ForceTorqueSensorInterface ft_interface;
  std::array<double, 3> force;
  std::array<double, 3> torque;
  std::string base_link, ee_link;
  // chain and jac_solver must have the same lifetime
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Jacobian jacobian;
  KDL::JntArray joint_positions_kdl;
  // low pass filtering
  std::vector<double> unfiltered_effort;
  double effort_pt1_T, velocity_pt1_T;
};
}  // namespace lbrmed_hw_fri