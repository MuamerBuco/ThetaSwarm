add_library( robotControl autoMR.cpp )

add_subdirectory(RobotCommands)
add_subdirectory(TrajectoryPlanning)

target_link_libraries( robotControl robotCommands )
target_link_libraries( robotControl trajectoryPlanning )

