add_library(feedback feedback.cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries (feedback Eigen3::Eigen)

target_link_libraries(feedback robotCommands)
target_link_libraries(feedback kinematics)
target_link_libraries(feedback pathPlanning)