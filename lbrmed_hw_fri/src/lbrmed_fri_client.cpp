// @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2022, Institute of Automatic Control - RWTH Aachen University
// All rights reserved.

#include <lbrmed_hw_fri/lbrmed_fri_client.hpp>
#include <stdexcept>
#include <utility>
#include <ros/ros.h>

#include <fri/friLBRCommand.h>
#include <fri/friLBRState.h>
#include <unistd.h>

namespace lbrmed_hw_fri
{
LBRMedFRIClient::LBRMedFRIClient(std::string name, std::vector<double>& joint_position,
                                 std::vector<double>& joint_effort, std::vector<double>& joint_position_command,
                                 std::vector<double>& joint_effort_command)
  : KUKA::FRI::LBRClient()
  , name(std::move(name))
  , joint_position(joint_position)
  , joint_effort(joint_effort)
  , joint_position_command(joint_position_command)
  , joint_effort_command(joint_effort_command)
{
}

void LBRMedFRIClient::onStateChange(KUKA::FRI::ESessionState old_mode, KUKA::FRI::ESessionState new_mode)
{
  steps_since_change = 0;
  if (new_mode == KUKA::FRI::ESessionState::IDLE)
  {
    throw new std::runtime_error("FRI session has been closed by the robot");
  }
  ROS_INFO_STREAM_NAMED(name, "Switched FRI session state from " << session_state_string(old_mode) << " to "
                                                                 << session_state_string(new_mode));
}

void LBRMedFRIClient::monitor()
{
  steps_since_change++;
  read_state();
}

void LBRMedFRIClient::waitForCommand()
{
  steps_since_change++;
  read_state();
  KUKA::FRI::LBRClient::waitForCommand();
}

void LBRMedFRIClient::command()
{
  steps_since_change++;
  read_state();
  if (joint_position_command.empty() || get_reset_required())
  {
    // Avoid unwanted jerky movements by setting current position
    KUKA::FRI::LBRClient::command();
  }
  else
  {
    robotCommand().setJointPosition(joint_position_command.data());
  }
}

void LBRMedFRIClient::read_state()
{
  auto position = robotState().getMeasuredJointPosition();
  joint_position.assign(position, position + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  auto effort = robotState().getExternalTorque();
  joint_effort.assign(effort, effort + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
}

bool LBRMedFRIClient::get_reset_required()
{
  // command will steps_since_change++ once before the control loop hits update(...) again
  return steps_since_change < 2;
}

std::string LBRMedFRIClient::session_state_string(int mode)
{
  switch (mode)
  {
    case KUKA::FRI::ESessionState::IDLE:
      return "idle";
    case KUKA::FRI::ESessionState::MONITORING_WAIT:
      return "monitoring wait";
    case KUKA::FRI::ESessionState::MONITORING_READY:
      return "monitoring ready";
    case KUKA::FRI::ESessionState::COMMANDING_WAIT:
      return "commanding wait";
    case KUKA::FRI::ESessionState::COMMANDING_ACTIVE:
      return "commanding active";
    default:
      return "undefined";
  }
}

}  // namespace lbrmed_hw_fri
