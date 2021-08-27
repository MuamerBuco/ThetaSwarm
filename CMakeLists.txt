cmake_minimum_required(VERSION 3.16)

# set the project name
project(openSwarm)

# include Ncurses
find_package(Curses REQUIRED)
include_directories( ${CURSES_INCLUDE_DIR} )

# add the executable
add_executable( ${PROJECT_NAME} main.cpp )

# add subdirectories
add_subdirectory(Common)
add_subdirectory(ArucoTracking)
add_subdirectory(SwarmControl)

# set compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# link libraries
target_link_libraries( ${PROJECT_NAME} common )
target_link_libraries( ${PROJECT_NAME} arucoDetection )
target_link_libraries( ${PROJECT_NAME} swarmControl )

target_link_libraries( ${PROJECT_NAME} ${CURSES_LIBRARIES} )

