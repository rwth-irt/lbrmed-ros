cmake_minimum_required(VERSION 3.0.2)
project(lbrmed_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  message_generation
  std_msgs)

## Generate messages 
# add_message_files(DIRECTORY msg)
# add_service_files(DIRECTORY srv)
add_action_files(DIRECTORY action)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

# Generate dynamic reconfigure files
generate_dynamic_reconfigure_options(
  config/lbrmed_impedance_fri.cfg
)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime std_msgs)
