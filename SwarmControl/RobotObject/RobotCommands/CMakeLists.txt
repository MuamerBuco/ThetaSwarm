add_library(robotCommands sendRobotCommands.cpp)

add_subdirectory(Client)

target_link_libraries( robotCommands PUBLIC util )
target_link_libraries( robotCommands PUBLIC client )