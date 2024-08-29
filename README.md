[![CI](https://github.com/rwth-irt/lbrmed-ros/actions/workflows/ci.yml/badge.svg)](https://github.com/rwth-irt/lbrmed-ros/actions/workflows/ci.yml)
# KUKA LBR MED STACK
ROS package for the Kuka LBR MED R820 (14kg)
We support
- commanding the robot hardware via the KUKA FRI interface and `ros_control`.
- kinematic simulations, Gazebo was never stable enough for contact rich tasks.
- MoveIt support for the hardware and simulation.

- A kinematic simulation environment and the robot hardware.
However, contact forces tend to be unstable.

Please look at the READMEs in the specific hardware packages.
**When using the real robot, always keep a safe distance and set up your safety configuration as required**.

# Install
## Devcontainer
This is the easiest way to get started
When opening this repository in Visual Studio Code, you should be prompted to reopen it in a container.
On launch, the container will setup all required dependencies via rosinstall and rosdep.

This repository also ships **VS Code tasks** for:
- Building the workspace `catkin build workspace`
- Building the package of the current file only `catkin build package`
- Clean the catkin build files `catkin clean`

After starting the container, the lbrmed-ros package will be opened as **workspace**.
The folder structure is `/workspace/src/lbrmed-ros`, so you will want to `cd` to a top-level folder when using low-level catkin commands

## Manual steps
### Create a catkin workspace
* Create a catkin workspace folder (e.g. ~/catkin_workspace)
* Create a src folder in that workspace (e.g. ~/catkin_workspace/src)
* Initialize the catkin workspace (e.g. `cd ~/catkin_workspace && catkin init`)

### Cloning the repository
We use [`git-lfs`](https://packagecloud.io/github/git-lfs/install) which should be installed before cloning.
Otherwise, you might get errors when the STL files of the robot or the jar files are being loaded.

For the Sunrise Project you have two ways to proceed:
* (a) Clone the repository into the src folder of the catkin workspace
* (b) (preferable) Clone the repository into a separate folder (e.g. ~/git-repos) and symlink it (e.g. `ln -s ~/git-repos/kuka_MED_Stack ~/catkin_workspace/src`, the target of the symlink must be an absolute path) 

### Installing Dependencies
In the catkin workspace, run the following commands to install all workspace and system dependencies.
```bash
wstool init src
wstool merge -t src src/lbrmed-ros/.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src -r -y
``` 

### Building the workspace
To build the packages in parallel, we use the [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html). After installing them, run:
```bash
cd ~/catkin_workspace
catkin build
```

# LBR MED Stack - Package Organization
*Reference: https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/#Simulation_with_Gazebo*

* lbrmed: Metapackage (optional)
* [lbrmed_bringup](./lbrmed_bringup): Main entry point launch files for this robot application, including Parameters and Configuration files. Good starting point to setup your own robot.
* [lbrmed_control](./lbrmed_control): Custom controllers for the LBR Med, e.g., a kind of admittance controller and integration of the FZI cartesian controllers.
* [lbrmed_description](./lbrmed_description): All files that are necessary for visualization (URDFs, Meshes, test launch files, ...)
* [lbrmed_hw_fri](./lbrmed_hw_fri/) (recommended): Implements the `hardware_interface::RobotHW` which can be used by ROS control using the FRI real-time interface.
* [lbrmed_moveit_config](./lbrmed_moveit_config): Default MoveIt configuration for without an end-effector, created with the MoveIt wizard.
* [lbrmed_msgs](./lbrmed_msgs): All custom messages, services and actions for this robot
* [lbrmed_ros_java](./lbrmed_ros_java): Transfer this code to the robot via the Sunrise Workbench. Add your own station setup and safety configuration!
