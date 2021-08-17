add_library(robotCommands sendMotorCommands.cpp)

target_link_libraries( robotCommands common )
target_link_libraries( robotCommands client )