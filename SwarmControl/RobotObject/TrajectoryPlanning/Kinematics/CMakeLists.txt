add_library( kinematics kinematics.cpp )

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

target_include_directories( robotControl PUBLIC ${OMPL_INCLUDE_DIRS} )

target_link_libraries( kinematics PUBLIC Eigen3::Eigen)
target_link_libraries( kinematics PUBLIC util )
