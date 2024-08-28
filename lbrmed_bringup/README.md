# LBRMed Bringup
This is the main entry point for using the LBRMed stack and can be used as a template to launch a custom robot configuration.
At this package's core are two launch files.

## hardware.launch
This launch file is used to launch the robot on a hardware system.
It loads the URDF and the hardware interface, and starts the robot controller.
You can choose between the `fri` or `smartservo` depending on the `robot_app` which is executed on the KUKA Sunrise cabinet.

## simulation.launch
This launch file is used to simulate the robot using fake controllers as a kinematics simulation.
It loads the URDF and the simulation parameters, and starts the robot controller.
