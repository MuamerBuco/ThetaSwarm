add_library(motionControl feedback.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (motionControl Eigen3::Eigen)

target_link_libraries(motionControl motorCommands)
target_link_libraries(motionControl kinematics)
target_link_libraries(motionControl navigation)