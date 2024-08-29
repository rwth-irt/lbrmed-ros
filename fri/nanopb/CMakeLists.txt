cmake_minimum_required(VERSION 3.0.2)
project(nanopb)

find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS} include ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Define target
add_library(${PROJECT_NAME}
    src/pb_decode.c
    src/pb_encode.c
)

if(MSVC)
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            -DPB_SYSTEM_HEADER=<${PROJECT_NAME}/pb_syshdr_win.h>
            -DPB_FIELD_16BIT
            -DWIN32
            -DHAVE_STDINT_H
            -DHAVE_STDBOOL_H
    )
else()
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            -O2
            -Wall
            -DHAVE_SOCKLEN_T
            -DPB_SYSTEM_HEADER=<${PROJECT_NAME}/pb_syshdr.h>
            -DPB_FIELD_16BIT
            -DHAVE_STDINT_H
            -DHAVE_STDDEF_H
            -DHAVE_STDBOOL_H
            -DHAVE_STDLIB_H
            -DHAVE_STRING_H 
    ) 
endif()

# Add include directories for target
target_include_directories(${PROJECT_NAME} PUBLIC include PRIVATE src
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# Install
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}) 



