add_library( robotObject 
    autoMR.cpp
    udpClient.cpp 
    kinematics.cpp 
    navigation.cpp 
    sendRobotCommands.cpp 
)

find_package(Boost REQUIRED COMPONENTS program_options thread) 
if(Boost_FOUND)
    # target_link_directories(${PROJECT_NAME} PRIVATE ${Boost_LIBRARY_DIRS})
    target_include_directories(robotObject PUBLIC ${Boost_INCLUDE_DIRS}) 
    target_link_libraries(robotObject PUBLIC Boost::program_options Boost::thread)
    message("Found Boost in the current environment!\n")
else()
    message("Error: No Boost found.")
endif()


find_package(OpenGL REQUIRED COMPONENTS OpenGL GLX)
if(OPENGL_FOUND)
    target_include_directories( robotObject PUBLIC ${OPENGL_INCLUDE_DIRS} )
    # target_link_libraries( robotObject PUBLIC ${OPENGL_LIBRARIES} )
    target_link_libraries( robotObject PUBLIC OpenGL::GLX OpenGL::OpenGL)
    message("Found OpenGL in the current environment!\n")
else()
    message("Error: No OpenGL found.")
endif()


find_package(ompl REQUIRED)
if(OMPL_FOUND)
    target_include_directories( robotObject PUBLIC ${OMPL_INCLUDE_DIRS} )
    target_link_libraries( robotObject PUBLIC ${OMPL_LIBRARIES} )
    target_link_libraries( robotObject PUBLIC ${OMPLAPP_LIBRARIES} )
    message("Found OMPL in the current environment!\n")
else()
    message("Error: No OMPL found.")
endif()