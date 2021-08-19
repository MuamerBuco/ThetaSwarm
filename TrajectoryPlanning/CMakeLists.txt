add_library(trajectoryPlanning trajectoryPlanning.cpp)

add_subdirectory(Feedback)
add_subdirectory(Kinematics)
add_subdirectory(PathPlanning)

target_link_libraries( trajectoryPlanning kinematics )
target_link_libraries( trajectoryPlanning pathPlanning )
