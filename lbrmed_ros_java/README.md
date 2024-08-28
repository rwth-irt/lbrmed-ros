# Sunrise Setup
Create an empty Sunrise project with your own StationSetup.cat and SafetyConfigrutaiton.sconf.
Make sure that neither a `/src` nor `/ROSJavaLib` folder exists inside the Sunrise project.
To link the files from this project to your SunriseProject:
- Open an **administrator** PowerShell session.
- Enable executing scripts via `set-executionpolicy remotesigned`
- `cd` into the Sunrise project folder.
- Copy the path to the `lbrmed_ros_java`. We will call it `$LBRMED_ROS_JAVA` in the next step.
- In PowerShell type `. $LBRMED_ROS_JAVA\symlink.ps1` and hit enter.

You should have two new symbolic links in your SunrisePorject folder by now.
In the Sunrise Workbench go to the package explorer and hit F5 to refresh the files list.
The src and ROSJavaLib folders and their contents should be available now.
Now, select all the .jar files in the ROSJavaLib folder, right click, and choose ` Build Path -> Add to Build Path`.

Congratulations, you should be able to sync to your Sunrise cabinet now.

# Apps
- `FRIServerPosition`: Control the robot in real-time using the position control mode.
  This is a **good default** if your environment allows for admittance control and does not require the impedance of the robot. 
- `FRIServerImpedance`: Control the robot in real-time using a joint impedance control mode.
  The stiffness of the joints is very high, so MoveIt does not complain.
  **Do NOT use with the FZI Cartesian controllers** which will cause oscillations of the robot.

# Station Setup and Safety Configuration
- We do not provide the station setup as hardware and software configurations will vary. 

## Station Setup
You...
  - must activate the `Fast Robot Interface Extension`
  - must activate your robot package, e.g., `LBR Med`
  - must activate the `Robotics API`
  - should enable the `Ikarus AntiVirus`
  - should enable the `<LBR Med> example applications` which contains break tests, etc.
  - might (have to) enable more packages

## Safety Configuration
We cannot know your safety requirements and you must set them yourself.
These are only some suggestions which make the operation safer without any liability.
You...
 - should set limits below slightly below the hardware limit of each axis' joint angle
 - should monitor and limit the Cartesian speed. This will limit the impulse during a collision.
 - should monitor and limit the TCP forces and torques
 - should set up a safe workspace volume

# Tool
- We provide the `DefaultTool` which is used in the `FRIServerPosition` and `FRIServerImpedance` apps.
- Configure and calibrate your own tool before using the apps, so the safety configuration can consider it.

# Message Generation
If you change the message definitions in lbrmed_msgs, please follow the instructions in the README of [lbrmed_msgs](lbrmed_msgs) to regenerate the Java messages.
It is important that the `lbrmed_msgs` and its dependencies are copied!
