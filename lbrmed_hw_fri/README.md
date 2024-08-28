# LBRMed Hardware FRI
ROS packages for the KUKA LBR, including communication to the real robot via the Fast Robot Interface ([FRI](https://github.com/KCL-BMEIS/fri)).

# Safety Information
The robot might execute **jerky motions** when starting the FRI motion, switching controllers, or if the commands are not sent in real-time, which could **injure** someone.
I observed this behavior specifically after emergency stops, which cause a difference between the measured positions and the interpolated commands of the robot's real-time system.
**Always keep a safe distance**, specifically during the aforementioned causes.

The following features have been implemented to minimize the amount of jerky motions.
* The robot drives to the mechanical zero position using a PTP motion when starting the program before opening the FRI connection. You have to confirm the PTP motion on the SmartPad to avoid accidental collisions.
* ROS controllers are reset after each change of FRI modes. Position saturation & soft limits are reset, too, since they have an internal state and override the controller values.
* The current measured joint positions are sent as commands once while resetting the controllers.

# DO NOT TRY
* ... to use the FZI Cartesian controllers with the FRIServerImpedance app, as it leads to oscillations. Use the FRIServerPosition app, instead.
* ... to change the stiffness / control mode via TCP calls - new connections lead to jitter in the communication and jerky motions with FRI running in parallel.
  I invested several days and could not figure it out reliably.

# Nodes
## lbrmed_hw_fri_node
Initializes a `ros_control::HardwareInterface` via the FRI interface of a KUKA LBR iiwa/Med.
The Sunrise program initializes a default configuration for the joint stiffness of *[1000, 1000, 1000, 500, 500, 500, 500]Nm/rad* and *0.7* for the damping ratio.
The ROS node establishes a TCP connection to enable changing the impedance control mode parameters.
After establishing the TCP connection, the FRI client connects to the Sunrise cabinet and the robot can be moved.
The TCP connection must be established before moving the robot since jerky motions occur when connecting new TCP sockets during an FRI session.
Thus, the whole node and the Sunrise program are killed if either the TCP or the FRI connection is lost.

The **Position Joint Interface** is implemented using [ros_control_boilerplate](http://wiki.ros.org/ros_control_boilerplate).
We use our own [fork](https://github.com/Tuebel/ros_control_boilerplate/tree/combined_robot_hw_draft), which allows resetting the controllers when the FRI connection state changes.
This prevents unpredictable jerky motions due to the controllers trying to command too large steps when the motion is stopped without them noticing.
Note, that the torques reported are the ones that KUKA call **external torques**, which does not include the typical terms like gravity acting on the links.

Moreover, the **Force-torque sensor interface** publishes the least squares estimate of the external Cartesian forces in the robot base frame.
Note, that this estimate is not as accurate as an external force-torque-sensor.
The developers of Drake observed an [accuracy of approximately 5N](https://github.com/achuwilson/pydrake-manipulator-documentation#estimating-cartesian-forces).

## lbrmed_hw_fri_dynamic_reconfig
Calls the `set_impedance_fri` service of the `lbrmed_hw_fri_node` to set the stiffness and damping of the joints from dynamic reconfigure.

## Acknowledgment
Based on [lbr_fri_ros2_stack](https://github.com/KCL-BMEIS/lbr_fri_ros2_stack/tree/noetic) which has a noetic branch.

# Setup
For the internal irt setup take a look at the [gitlab wiki](https://git-ce.rwth-aachen.de/g-med-irt-robotik/lbrmed-stack/-/wikis/Robot%20Hardware%20Usage) and the [irt wiki](https://wiki.irt.rwth-aachen.de/wiki/KUKA_LBR).

## Setup the Controller
The controller (Sunrise Cabinet) receives commands from the ROS machine via the FRI. Therefore, the server application has to be pushed onto the Sunrise Cabinet.
- Connect an ethernet cable to port X66 on the Sunrise Cabinet
- By default, the controller's IP address is `172.31.1.147`, ping the KUKA Sunrise Cabinet
## Synchronize the Server Application
To push the server application that handles the communication to the robot
 - Copy the contents of [server](server) to the `src` folder inside the Sunrise project 
 - In the Software tab of the StationSetup.cat, tick the `Fast Robot Interface Extension` box
 - Install settings to the controller, in the Installation tab of the StationSetup.cat, press install
 - Synchronize the Sunrise project

# IP-Settings
If you would like to talk to your robot, make sure that the computer and the robot are in the same subnet. The default settings are the following static IPs:

| Device | IP-Address | Subnet Mask | 
| -------|------------|-------------|
| Robot | 172.31.1.147 | FFFFFF00 |
| Computer | 172.31.1.150 | 255.255.255.0 |

# Notes for Windows Subsystem for Linux
* use the new mirrored networking mode
* open Windows firewall UDP port 30200 for incoming connections
