# Install script for directory: /home/eon/OpenSwarm/3rdParty/ompl-1.5.2

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/ompl.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/cmake" TYPE FILE FILES
    "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/omplConfig.cmake"
    "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/omplConfigVersion.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl" TYPE FILE FILES "/home/eon/OpenSwarm/3rdParty/ompl-1.5.2/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/share/ament_index/resource_index/packages/ompl")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/src/cmake_install.cmake")
  include("/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/py-bindings/cmake_install.cmake")
  include("/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/tests/cmake_install.cmake")
  include("/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/demos/cmake_install.cmake")
  include("/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/scripts/cmake_install.cmake")
  include("/home/eon/OpenSwarm/build/3rdParty/ompl-1.5.2/doc/cmake_install.cmake")

endif()

