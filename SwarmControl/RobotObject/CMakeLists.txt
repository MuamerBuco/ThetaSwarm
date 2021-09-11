add_library( robotControl autoMR.cpp )

add_subdirectory(RobotCommands)
add_subdirectory(TrajectoryPlanning)

find_package(ompl REQUIRED)

include_directories( robotControl PUBLIC ${OMPL_INCLUDE_DIRS} )

target_link_libraries( robotControl ${OMPL_LIBRARIES} )
target_link_libraries( robotControl robotCommands )
target_link_libraries( robotControl trajectoryPlanning )
