
# LBR MED MoveIt Config
This package contains the configuration files for the LBR Med robot, enabling MoveIt to generate trajectories for the robot.

## Overview
The package includes the following files:

* `launch`: start MoveIt with the LBR Med robot configuration.
* `config`: MoveIt configuration files including the robot description, joint limits, end-effector constraints, and planners.
* `srdf`: [Soft Robot Description Format](https://moveit.ros.org/documentation/concepts/srdf/) for the LBR Med robot.

## Usage
You should build on top of the `move_group.launhch` file.
