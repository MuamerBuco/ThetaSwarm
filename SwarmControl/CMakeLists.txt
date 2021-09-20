add_library( swarmControl swarm.cpp )

add_subdirectory(RobotObject)
add_subdirectory(PathPlanning)

target_link_libraries( swarmControl PUBLIC robotControl )
target_link_libraries( swarmControl PUBLIC pathPlanning )
