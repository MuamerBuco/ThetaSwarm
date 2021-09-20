cmake_minimum_required(VERSION 3.16)
set (CMAKE_CXX_STANDARD 17)

# set the project name
project(openSwarm)

# add the executable
add_executable( ${PROJECT_NAME} main.cpp )

# add subdirectories
add_subdirectory(util)
add_subdirectory(ArucoTracking)
add_subdirectory(SwarmControl)

# link libraries
target_link_libraries( ${PROJECT_NAME} PUBLIC util )
target_link_libraries( ${PROJECT_NAME} PUBLIC arucoDetection )
target_link_libraries( ${PROJECT_NAME} PUBLIC swarmControl )
