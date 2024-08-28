cmake_minimum_required(VERSION 2.8.3)
project(fri)

find_package(catkin REQUIRED COMPONENTS nanopb)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fri_lib
    CATKIN_DEPENDS nanopb
)
include_directories(${catkin_INCLUDE_DIRS} include ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Define target
add_library(fri_lib
    src/friClientApplication.cpp
    src/friCommandMessageEncoder.cpp
    src/friLBRClient.cpp
    src/friLBRCommand.cpp
    src/friLBRState.cpp
    src/FRIMessages.pb.c
    src/friMonitoringMessageDecoder.cpp
    src/friTransformationClient.cpp
    src/friUdpConnection.cpp
    src/pb_frimessages_callbacks.c
)

# Add compile options
if(MSVC)
    target_compile_options(fri_lib
        PRIVATE -DPB_SYSTEM_HEADER=<nanopb/pb_syshdr_win.h>
            -DPB_FIELD_16BIT
            -DWIN32
            -DHAVE_STDINT_H
            -DHAVE_STDBOOL_H
    )
else()
    target_compile_options(fri_lib
        PRIVATE
            -O2
            -Wall
            -DHAVE_SOCKLEN_T -DPB_SYSTEM_HEADER=<nanopb/pb_syshdr.h>
            -DPB_FIELD_16BIT
            -DHAVE_STDINT_H
            -DHAVE_STDDEF_H
            -DHAVE_STDBOOL_H
            -DHAVE_STDLIB_H
            -DHAVE_STRING_H 
    ) 
endif()

# Add include directories for target
target_include_directories(fri_lib PUBLIC include PRIVATE src)

# Link against dependencies
target_link_libraries(fri_lib
    ${catkin_LIBRARIES}
)

# Install
install(TARGETS fri_lib  
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}) 
