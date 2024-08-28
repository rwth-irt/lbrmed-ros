# LBRMED Description
## config/launch
The files located in the config and launch folder are for testing purposes only. 
Files for uploading the lbrmed description onto the parameter server and for visualizing
it in e.g. RViz are located in the *lbrmed_bringup* package. 

## meshes
This folder contains the meshes used for visualization and collision in RViz and Gazebo.
The meshes are STL files and were processed with Blender. The coordinate systems are oriented such
that the z-axis points in the direction of the rotational axis of the corresponding joint. All x-axes
point in the same direction for all meshes. Information and tips on how to manipulate meshes with Blender
can be found in the wiki of the git repo.

## urdf
This folder contains all urdf and xacro files for the description, visualization and additional information
for other nodes directly related to the lbrmed. 

### lbrmed14.xacro

- **Content:** xacro macro of all kinematic, collision and visual information about the lbrmed14 
- **Parameter:** 
    - `connected_to:` with this parameter the base (link0) frame of the robot can be connected to another frame outside the lbrmed description (e.g. world)
    - `arm_id:` the robot links and joints follow the **naming convention** *arm_id_link_x* or *arm_id_jointx* where *x* stands for the index of the link or joint. Changing this parameter from the default *lbrmed14*, changes the joint names which in turn has to be considered for all nodes that use those names (e.g. moveit, controllers, etc.)
    - `xyz:` is a vector of three real numbers in [m] and describes the translation from the *connected_to* frame to the base (link_0) frame of the robot. This parameter is only used if the base frame is connected to another frame.
    - `rpy:` is a vector of three real numbers in [rad] and describes the rotation from the *connected_to* frame to the base (link_0) frame of the robot. This parameter is only used if the base frame is connected to another frame.
- **Coordinate Systems:**
    - [`Links:`](http://wiki.ros.org/urdf/XML/link) Whenever a link is created, a coordinate system is created that serves as a reference for all tags described with it and for the joints where this link is the parent.
        - visual: defines the visual mesh of that link. The origin tag describes the translation and rotation between the cs of the link and the cs defined in the mesh itself.
        - collision: defines the collision mesh of that link. The origin tag describes the translation and rotation between the cs of the link and the cs defined in the mesh itself.
        - material: specifies the color of the meshes in RViz. They are defined in *materials.xacro*
        - inertia: defines inertia properties of link including the mass. The orgin tag describes the translation and rotation between the cs of the link and the center of mass coordinate system. The inertia matrix is symmetric.
    - [`Joints:`](http://wiki.ros.org/urdf/XML/joint) The lbrmed has 7 revolute joints (*robot_name_joint1* to *robot_name_joint7*) and two fixed joints. One connects the base (link0) to an outside frame and one connects link7 to the end effector link (link8)
        - parent/child: these are the links that are connected by the joint
        - origin: describes the rotation and translation of the joint regarding the parent link cs. The child link cs will then be placed so that it coincides with the joint cs.
        - axis: axis of rotation for the revolute joints in the joint cs. The coordinate systems are laid out such that the positive z-axis is always the axis of rotation.
        - limits: describes the limits (rotation, effort, velocity) of the joint. Used in Gazebo and MoveIt.
        - damping: dynamic properties of the joint. Used in Gazebo.
        - safety_controller: properties for the safety controller.
    - End effector-Link (link8): This link has no properties and just acts as an interface to connect tools to. Moreover, it serves as a dummy for the Gazebo force-torque-sensor in joint8.

### utilities.xacro

Contains miscellaneous information, like constants or simple inertia properties.

### materials.xacro

Contains definitions of different materials or colors, that can be used in the visualization in Rviz.

### lbrmed.transmission.xacro

- **Content:** Contains the xacro macro *lbrmed_transmission* that defines [transmission](http://wiki.ros.org/urdf/XML/Transmission) properties of all joints and the corresponding hardware interface. This information is needed by the *ros_control* package.
- **Parameter:** 
    - `arm_id:` the transmission, the actuator and the joints use the same naming convention used in the lbrmed macro (*arm_id_tran_x*, *arm_id_moter_x*, *robot_name_jointx*).
    - `hardware_interface:` defines the hardware interface of this joint, which determines what input signals the actuator can receive.The hardware interface has to match the controller output.

### lbrmed.gazebo.xacro

Contains additional information about the robot and links used in gazebo. 
More information can be found [here](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros). 
It also loads the _gazebo\_ros\_control_ plugin and the simulated hardware interface that is used in combination with the *ros_control* package.
The 'libgazebo_ros_ft_sensor* plugin is used to publish the end effector wrenches.

### lbrmed.urdf.xacro

This is a wrapper file that loads all necessary macros and sets their parameters. It also creates an empty _world_ link that can serve as a fixed reference in gazebo and Rviz for visualization and testing. 
