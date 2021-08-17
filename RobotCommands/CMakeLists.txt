add_library(motorCommands manualControl.cpp sendMotorCommands.cpp)

target_link_libraries( motorCommands common )
target_link_libraries( motorCommands client )