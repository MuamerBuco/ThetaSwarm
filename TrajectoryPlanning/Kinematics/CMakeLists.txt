add_library( kinematics kinematics.cpp )

target_link_libraries( kinematics feedback )
target_link_libraries( kinematics common )
