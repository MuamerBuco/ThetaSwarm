add_library(trajectoryPlanning trajectoryPlanning.cpp)

add_subdirectory(Kinematics)


target_link_libraries( trajectoryPlanning PUBLIC kinematics )

