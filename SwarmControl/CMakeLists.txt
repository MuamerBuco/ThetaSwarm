add_library( swarmControl swarm.cpp )

add_subdirectory(RobotControl)
add_subdirectory(MotionPlanning)

target_link_libraries(swarmControl PUBLIC robotControl motionPlanning)