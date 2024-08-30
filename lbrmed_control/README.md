# LBRMed Control
Configurations of ros controllers which can be used with the hardware and simulation.
Provides launch files to load and start a set of default controllers.

## config
- lbrmed_control.yaml: Default configuration of controllers.
- lbrmed_sim_hw.yaml: Configuration of fake controllers for the kinematic simulation via `ros_control_boilerplate`.

## launch
- lbrmed_control.launch: Launch file to load and start a set of default controllers.
  - `controllers`: List of controllers to start immediately.
  - `load_controllers`: List of controllers to load but not start.
- lbrmed_hw_sim.launch: Launch file to launch the `ros_control_boilerplate` fake controllers.

## Typically, used controllers
- `PositionJointInterface_trajectory_controller`: Implements a controller which accepts trajectories of joint values, e.g. from MoveIt.
- `PositionJointInterface_cartesian_motion_controller`: Implements a controller which accepts Cartesian pose commands.
- `joint_state_controller`: Implements a controller which re-publishes the joint states of the robot.
- `force_torque_sensor_controller`: Implements a controller which re-publishes the force and torque sensors of the robot.

## FZI Cartesian controllers
The config contains entries for the [FZI Cartesian controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).
However, this package is intended to be a self-contained build and source dependencies are reduced to a minimum.
To install the Cartesian controllers, run the following commands in the workspace root:
```shell
wstool init src
wstool merge -t src src/lbrmed-ros/lbrmed_control/.rosinstall
wstool update -t src
```
Then re-build and -source the whole workspace

## Custom controllers
- `PositionJointInterface_abs_admittance_controller`: Implements an absolute force-based admittance controller.
Compared to a normal admittance controller, the robot approaches an object until the total force exceeds a limit and stops.
I tried implementing a similar behavior using the [FZI Cartesian controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers), but it deviated from the commanded position, no matter how I tuned the parameters.
This controller uses good old KDL inverse kinematics and monitors the forces in the ROS control loop.
Compared to the FZI version, this implementation is **less robust near singularities**.
  - Offers `abs_admittance_control` action:
    - Send a goal velocity `velocity` and absolute goal force `goal_abs_force`
    - Robot will measure use the currently measured Cartesian force as a static tare force which is used with the `force_limit` for safety
    - Robot will start approaching with the constant goal velocity
    - after `dynamic_tare_time` the robot will measure the current Cartesian force as a dynamic tare force. The measured joint torques change due to dynamic friction compared to the static tare force.
    - If the absolute force abs(tare - measured) exceeds the goal or the safety limit, the robot will stop.
  - Parameters in *config/lbrmed_control.yaml*
    - `force_limit` is a safety limit used with the static tare measurement
    - `dynamic_tare_time` is the time in seconds after which the dynamic tare force is measured