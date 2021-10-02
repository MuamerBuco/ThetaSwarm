add_library( swarmControl swarm.cpp )

add_subdirectory(src)

target_link_libraries(swarmControl PUBLIC robotObject)