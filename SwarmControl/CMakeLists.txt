add_library( swarmControl swarm.cpp )

add_subdirectory(RobotObject)

target_link_libraries( swarmControl robotControl )