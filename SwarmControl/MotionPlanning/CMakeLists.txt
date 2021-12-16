add_library( motionPlanning 
    src/kinematics.cpp 
    src/pathPlanning.cpp 
)

find_package(Boost REQUIRED COMPONENTS program_options thread) 
if(Boost_FOUND)
    # target_link_directories(${PROJECT_NAME} PRIVATE ${Boost_LIBRARY_DIRS})
    target_include_directories(motionPlanning PUBLIC ${Boost_INCLUDE_DIRS}) 
    target_link_libraries(motionPlanning PUBLIC Boost::program_options Boost::thread)
    message("Found Boost in the current environment!\n")
else()
    message("Error: No Boost found.")
endif()


find_package(OpenGL REQUIRED COMPONENTS OpenGL GLX)
if(OPENGL_FOUND)
    target_include_directories( motionPlanning PUBLIC ${OPENGL_INCLUDE_DIRS} )
    # target_link_libraries( motionPlanning PUBLIC ${OPENGL_LIBRARIES} )
    target_link_libraries( motionPlanning PUBLIC OpenGL::GLX OpenGL::OpenGL)
    message("Found OpenGL in the current environment!\n")
else()
    message("Error: No OpenGL found.")
endif()


find_package(ompl REQUIRED)
if(OMPL_FOUND)
    target_include_directories( motionPlanning PUBLIC ${OMPL_INCLUDE_DIRS} )
    target_link_libraries( motionPlanning PUBLIC ${OMPL_LIBRARIES} )
    target_link_libraries( motionPlanning PUBLIC ${OMPLAPP_LIBRARIES} )
    message("Found OMPL in the current environment!\n")
else()
    message("Error: No OMPL found.")
endif()