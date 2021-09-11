add_library(robotCommands sendRobotCommands.cpp)

add_subdirectory(Client)

target_link_libraries( robotCommands common )
target_link_libraries( robotCommands client )