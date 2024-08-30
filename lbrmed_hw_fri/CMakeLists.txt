cmake_minimum_required(VERSION 3.1)
project(lbrmed_hw_fri)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Used for asio
find_package(Boost REQUIRED system)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# Because of licensing
add_definitions(-DEIGEN_MPL2_ONLY)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  fri
  kdl_parser
  lbrmed_msgs
  pluginlib
  ros_control_boilerplate
  roscpp
  rosparam_shortcuts
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    dynamic_reconfigure
    fri
    kdl_parser
    lbrmed_msgs
    pluginlib
    ros_control_boilerplate
    roscpp
    rosparam_shortcuts
)
include_directories(${catkin_INCLUDE_DIRS})

# Libraries
add_library(${PROJECT_NAME}
  src/lbrmed_fri_client.cpp
  src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

# Nodes
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Install Header
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}) 

# Install libraries
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install executables
install(TARGETS ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############
# Unit Tests
############

find_package(GTest REQUIRED)
# include(GoogleTest)

if (CATKIN_ENABLE_TESTING)
  # unit testing with google test
  # add all sources from the test/ directory as test
  file(GLOB TEST_SRCS test/*.cpp)
  foreach(sourcefile ${TEST_SRCS})
    get_filename_component(testname ${sourcefile} NAME_WE)
    add_executable(${testname} ${sourcefile})
    target_link_libraries (${testname}
      ${PROJECT_NAME} GTest::GTest GTest::Main)
    gtest_discover_tests(${testname})
  endforeach()
endif()

