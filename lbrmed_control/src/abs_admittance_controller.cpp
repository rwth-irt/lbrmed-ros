// @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
// All rights reserved.

#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <lbrmed_control/abs_admittance_controller.hpp>

namespace lbrmed_control
{

void AbsAdmittanceController::starting(const ros::Time& time)
{
  // Reset elapsed time to start a new approach movement
  motion_seconds_ = 0.0;
  velocity_ = 0.0;
  goal_abs_force_ = 0.0;
  dynamic_tare_set_ = false;
  // Read current cartesian pose
  auto joint_states = get_joint_states();
  KDL::ChainFkSolverPos_recursive fk_solver_pos(kdl_chain_);
  auto fk_result = fk_solver_pos.JntToCart(joint_states.q, start_pose_);
  if (fk_result < 0)
  {
    ROS_ERROR_STREAM_NAMED("AbsAdmittanceController", "Failed to get start pose");
    throw std::runtime_error("Failed to get start pose");
  }
  // Init tare force
  static_tare_force_ = wrench_in_base(joint_states).head(3);
  ROS_INFO_STREAM_NAMED("AbsAdmittanceController",
                        "AbsAdmittanceController static tare force: " << static_tare_force_ << std::endl);
}

void AbsAdmittanceController::update(const ros::Time& time, const ros::Duration& period)
{
  // stop early if action not active
  if (!action_server_->isActive())
  {
    return;
  }

  // Get current state
  auto joint_states = get_joint_states();
  auto measured_wrench = wrench_in_base(joint_states);
  auto measured_force = measured_wrench.head(3);

  // Stop if absolute force exceeds safety limit
  if ((measured_force - static_tare_force_).norm() > force_limit_)
  {
    // Stop motion after exceeding force limit
    ROS_ERROR_NAMED("AbsAdmittanceController", "Force safety limit exceeded: %fN", force_limit_);
    stopping(time);
  }

  // Set dynamic tare force at least once for motion time = 0
  if (motion_seconds_ <= dynamic_tare_time_)
  {
    dynamic_tare_force_ = measured_force;
  }
  else
  {
    // Info once
    if (!dynamic_tare_set_)
    {
      ROS_INFO_STREAM_NAMED("AbsAdmittanceController",
                            "AbsAdmittanceController dynamic tare force:" << dynamic_tare_force_ << std::endl);
    }
    dynamic_tare_set_ = true;
  }

  // Calculate tared absolute force and publish feedback
  auto abs_force = (measured_force - dynamic_tare_force_).norm();
  lbrmed_msgs::AbsAdmittanceControlFeedback feedback;
  feedback.header.stamp = time;
  feedback.header.frame_id = base_link_;
  feedback.abs_force = abs_force;
  action_server_->publishFeedback(feedback);
  // For reproducibility
  publish_6d_wrenches(measured_wrench, time);

  // Keep moving only if the fore is below the goal force
  if (abs_force < goal_abs_force_)
  {
    // Only add to elapsed time if not stopped, otherwise we would have large steps
    motion_seconds_ += period.toSec();
    // Cartesian position displacement for constant velocity in tool frame z-direction
    KDL::Frame displacement(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0, 0, motion_seconds_ * velocity_));
    // Goal pose calculated relative to start pose to avoid drifting
    // which might occur if we use the current pose + a small delta
    KDL::Frame goal_pose = start_pose_ * displacement;

    // Setup inverse kinematics
    KDL::ChainFkSolverPos_recursive fk_solver_pos(kdl_chain_);
    KDL::ChainIkSolverVel_wdls ik_solver_vel(kdl_chain_, 1.0E-6);
    KDL::ChainIkSolverPos_NR ik_solver_pos(kdl_chain_, fk_solver_pos, ik_solver_vel, 200);

    // Calculate goal pose using the current joint positions as initial guess, which should be close to goal
    KDL::JntArray goal_joint_positions(joints_.size());
    auto ik_result = ik_solver_pos.CartToJnt(joint_states.q, goal_pose, goal_joint_positions);
    if (ik_result < 0)
    {
      ROS_ERROR_STREAM("Could not calculate joint pose, error code: " << ik_result << std::endl);
      return;
    }
    // Write goal pose to joint position handles
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      joints_[i].setCommand(goal_joint_positions(i));
    }
  }
  else
  {
    lbrmed_msgs::AbsAdmittanceControlResult result;
    result.max_abs_force = abs_force;
    action_server_->setSucceeded(result);
    ROS_INFO_NAMED("AbsAdmittanceController", "AbsAdmittance control succeeded");
    stopping(time);
  }
}

void AbsAdmittanceController::stopping(const ros::Time&)
{
  // Reset force and velocity limits to default to stand still
  velocity_ = 0.0;
  goal_abs_force_ = 0.0;
  // Abort action if controller is forced to shutdown, alternative is preempt by client
  if (action_server_->isActive())
  {
    action_server_->setAborted();
  }
}

Eigen::Matrix<double, 6, 1> AbsAdmittanceController::wrench_in_base(const KDL::JntArrayAcc& joint_states)
{
  // Calculate jacobian for current positions
  KDL::ChainJntToJacSolver jac_solver(kdl_chain_);
  KDL::Jacobian jacobian(kdl_chain_.getNrOfJoints());
  jac_solver.JntToJac(joint_states.q, jacobian);
  // inverse(jacobian^T) * joint_torques
  return jacobian.data.transpose().completeOrthogonalDecomposition().solve(joint_states.qdotdot.data);
}

KDL::JntArrayAcc AbsAdmittanceController::get_joint_states()
{
  KDL::JntArrayAcc joint_states(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    joint_states.q(i) = joints_[i].getPosition();
    joint_states.qdot(i) = joints_[i].getVelocity();
    joint_states.qdotdot(i) = joints_[i].getEffort();
  }
  return joint_states;
}

void AbsAdmittanceController::goal_action_callback()
{
  auto goal = action_server_->acceptNewGoal();
  goal_abs_force_ = goal->goal_abs_force;
  velocity_ = goal->velocity;
  ROS_INFO_STREAM_NAMED("abs_admittance_controller", "Accepted goal with force limit: " << goal_abs_force_
                                                                                        << ", velocity: " << velocity_
                                                                                        << std::endl);
}
void AbsAdmittanceController::preempt_action_callback()
{
  // Stop motion
  stopping(ros::Time::now());
  action_server_->setPreempted();
}

bool AbsAdmittanceController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  ROS_INFO_NAMED("AbsAdmittanceController", "Initializing AbsAdmittanceController (ns: %s)", nh.getNamespace().c_str());
  // Read force limit from parameter server
  if (!nh.getParam("force_limit", force_limit_))
  {
    ROS_ERROR_NAMED("AbsAdmittanceController", "No force limit given");
    return false;
  }
  // Read time until dynamic tare is set
  if (!nh.getParam("dynamic_tare_time", dynamic_tare_time_))
  {
    ROS_ERROR_NAMED("AbsAdmittanceController", "No tare time given defaulting to 0.0 seconds");
    dynamic_tare_time_ = 0.0;
  }
  // Init ROS control resources
  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR_NAMED("AbsAdmittanceController", "No joints given");
    return false;
  }
  for (const auto& joint_name : joint_names)
  {
    joints_.push_back(hw->getHandle(joint_name));
  }
  // Init KDL tree for calculation of force and IK solution
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR_NAMED("AbsAdmittanceController", "Failed to parse URDF model form 'robot_description'");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
  {
    ROS_ERROR_NAMED("AbsAdmittanceController", "Failed to construct kdl tree from URDF");
    return false;
  }
  if (joint_names.size() != kdl_tree_.getNrOfJoints())
  {
    ROS_ERROR_NAMED("AbsAdmittanceController", "Number of joints in parameter does not match kdl tree");
    return false;
  }
  // Init KDL chain from base to end effector
  std::string end_effector_link;
  nh.param<std::string>("end_effector_link", end_effector_link, "lbrmed14_link8");
  nh.param<std::string>("robot_base_link", base_link_, "lbrmed14_link0");
  if (!kdl_tree_.getChain(base_link_, end_effector_link, kdl_chain_))
  {
    ROS_ERROR_NAMED("AbsAdmittanceController", "Failed to construct kdl chain from base link to end effector link");
    return false;
  }
  // action server
  action_server_ = std::make_unique<actionlib::SimpleActionServer<lbrmed_msgs::AbsAdmittanceControlAction>>(
      nh, "abs_admittance_control", false);
  action_server_->registerGoalCallback([this]() { goal_action_callback(); });
  action_server_->registerPreemptCallback([this]() { preempt_action_callback(); });
  action_server_->start();
  // Publisher
  wrench_publisher_ = nh.advertise<geometry_msgs::WrenchStamped>("abs_admittance_control/measured_wrench", 1);
  tare_publisher_ = nh.advertise<geometry_msgs::WrenchStamped>("abs_admittance_control/tare_wrench", 1);
  ROS_INFO_NAMED("AbsAdmittanceController", "AdmittanceController initialized, ready to receive actions");
  return true;
}

void AbsAdmittanceController::publish_6d_wrenches(Eigen::Matrix<double, 6, 1> measured_wrench, ros::Time stamp)
{
  geometry_msgs::WrenchStamped measured_wrench_msg;
  measured_wrench_msg.header.stamp = stamp;
  measured_wrench_msg.header.frame_id = base_link_;
  measured_wrench_msg.wrench.force.x = measured_wrench(0);
  measured_wrench_msg.wrench.force.y = measured_wrench(1);
  measured_wrench_msg.wrench.force.z = measured_wrench(2);
  measured_wrench_msg.wrench.torque.x = measured_wrench(3);
  measured_wrench_msg.wrench.torque.y = measured_wrench(4);
  measured_wrench_msg.wrench.torque.z = measured_wrench(5);
  wrench_publisher_.publish(measured_wrench_msg);

  geometry_msgs::WrenchStamped tare_wrench_msg;
  tare_wrench_msg.header.stamp = stamp;
  tare_wrench_msg.header.frame_id = base_link_;
  if (dynamic_tare_set_)
  {
    tare_wrench_msg.wrench.force.x = dynamic_tare_force_(0);
    tare_wrench_msg.wrench.force.y = dynamic_tare_force_(1);
    tare_wrench_msg.wrench.force.z = dynamic_tare_force_(2);
  }
  else
  {
    tare_wrench_msg.wrench.force.x = static_tare_force_(0);
    tare_wrench_msg.wrench.force.y = static_tare_force_(1);
    tare_wrench_msg.wrench.force.z = static_tare_force_(2);
  }
  tare_wrench_msg.wrench.torque.x = 0.0;
  tare_wrench_msg.wrench.torque.y = 0.0;
  tare_wrench_msg.wrench.torque.z = 0.0;
  tare_publisher_.publish(tare_wrench_msg);
}

}  // namespace lbrmed_control

PLUGINLIB_EXPORT_CLASS(lbrmed_control::AbsAdmittanceController, controller_interface::ControllerBase)
