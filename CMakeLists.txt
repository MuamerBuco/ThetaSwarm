cmake_minimum_required(VERSION 3.16)

# set the project name
project(udp_client)

# include Ncurses
find_package(Curses REQUIRED)
include_directories( ${CURSES_INCLUDE_DIR} )

# add the executable
add_executable( ${PROJECT_NAME} main.cpp )

# add subdirectories
add_subdirectory(Client)
add_subdirectory(Kinematics)
add_subdirectory(CommonFunctions)
add_subdirectory(ArucoTracking)
add_subdirectory(MotorCommands)
add_subdirectory(MotionControl)
add_subdirectory(Navigation)

# set compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# link libraries
target_link_libraries( ${PROJECT_NAME} client )
target_link_libraries( ${PROJECT_NAME} kinematics )
target_link_libraries( ${PROJECT_NAME} common )
target_link_libraries( ${PROJECT_NAME} arucoDetection )
target_link_libraries( ${PROJECT_NAME} motorCommands )
target_link_libraries( ${PROJECT_NAME} motionControl )
target_link_libraries( ${PROJECT_NAME} navigation )

target_link_libraries( ${PROJECT_NAME} ${CURSES_LIBRARIES} )

