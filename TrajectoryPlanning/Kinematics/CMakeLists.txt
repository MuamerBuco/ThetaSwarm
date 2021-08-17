add_library( kinematics kinematics.cpp )

add_subdirectory(Feedback)

target_link_libraries( kinematics feedback )
target_link_libraries( kinematics common )
