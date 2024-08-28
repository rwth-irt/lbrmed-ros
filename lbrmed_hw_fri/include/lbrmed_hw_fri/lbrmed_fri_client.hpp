// @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2022, Institute of Automatic Control - RWTH Aachen University
// All rights reserved.
#pragma once

#include <array>
#include <fri/friLBRClient.h>
#include <string>
#include <vector>

namespace lbrmed_hw_fri
{
/**
 * @brief Implements a LBRClient that communicates to the real robot via the Fast Robot Interface
 * @param lbr A shared pointer to a LBR object
 *
 * The LBRClient updates the state of the shared LBR object via update_lbr(), and sends desired
 * commands from the LBR object to the real robot via command(). The shared LBR object is also
 * accessed by the FriRos node and, therefore, to communicate to the real robot via ROS2.
 **/
class LBRMedFRIClient : public KUKA::FRI::LBRClient
{
public:
  /**
   * @brief Constructor to inject the robot state and command vectors of the hardware interface. Since this class is
   *owned by the hardware interface, we should not have any lifetime issues.
   * @param name The name of the robot
   **/
  LBRMedFRIClient(std::string name, std::vector<double>& joint_position, std::vector<double>& joint_effort,
                  std::vector<double>& joint_position_command, std::vector<double>& joint_effort_command);

  /**
   * @brief Callback on state change that prints the old and new state
   **/
  void onStateChange(KUKA::FRI::ESessionState old_mode, KUKA::FRI::ESessionState new_mode) override;

  /**
   * @brief Callback if the real robot is in KUKA::FRI::ESessionState MONITORING_READY
   **/
  void monitor() override;

  /**
   * @brief Callback if the real robot is in KUKA::FRI::ESessionState COMMANDING_WAIT
   **/
  void waitForCommand() override;

  /**
   * @brief Callback triggered by the FRI, writes the commands hardware interface to the real robot state via the Fast
   *Robot Interface.
   **/
  void command() override;

  /**
   * @brief Reads the real robot state via the Fast Robot Interface and stores it in the hardware interface state.
   **/
  void read_state();

  /**
   * @brief Reads the real robot state via the Fast Robot Interface and stores it in the hardware interface state.
   **/
  bool get_reset_required();

  /**
   * @brief Translates the robot's state to human readable strings
   * @param mode Integer from the KUKA::FRI::ESessionState enum
   **/
  std::string session_state_string(int mode);

private:
  std::string name;

  // References to the states and command of the HardwareInterface
  std::vector<double>& joint_position;
  std::vector<double>& joint_effort;
  std::vector<double>& joint_position_command;
  std::vector<double>& joint_effort_command;
  // Count the the number of steps since changing the mode to notify get_reset_required
  uint64_t steps_since_change = 0;
};
}  // namespace lbrmed_hw_fri
