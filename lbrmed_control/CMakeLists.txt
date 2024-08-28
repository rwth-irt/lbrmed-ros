cmake_minimum_required(VERSION 3.0.2)
project(lbrmed_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  controller_interface
  hardware_interface
  kdl_parser
  lbrmed_msgs
  pluginlib
  roscpp)

catkin_package(
 INCLUDE_DIRS include  
 LIBRARIES ${PROJECT_NAME}  
 CATKIN_DEPENDS
  actionlib
  controller_interface
  lbrmed_msgs
  pluginlib
  roscpp)

include_directories(${catkin_INCLUDE_DIRS})

# Libraries
add_library(${PROJECT_NAME}
  src/abs_admittance_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

# Install Header
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}) 

# Install libraries
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install Plugin
install(FILES ${PROJECT_NAME}_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
