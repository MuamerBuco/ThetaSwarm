# Install script for directory: /home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/KinematicChainPathPlot.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/OptimalPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/PlannerData.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/Point2DPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/RandomWalkPlanner.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/RigidBodyPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/RigidBodyPlanningWithControls.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/RigidBodyPlanningWithODESolverAndControls.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ompl/demos/StateSampling.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/CForestCircleGridBenchmark.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/Diagonal.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/GeometricCarPlanning.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/HybridSystemPlanning.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/HypercubeBenchmark.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/KinematicChainBenchmark.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/LTLWithTriangulation.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/OpenDERigidBodyPlanning.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/OptimalPlanning.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/PlannerData.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/PlannerProgressProperties.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/Point2DPlanning.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/RigidBodyPlanning.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/RigidBodyPlanningWithControls.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/RigidBodyPlanningWithIK.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/RigidBodyPlanningWithIntegrationAndControls.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/RigidBodyPlanningWithODESolverAndControls.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/StateSampling.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/ThunderLightning.cpp"
    "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/TriangulationDemo.cpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/Koules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/demos/VFRRT")
endif()

